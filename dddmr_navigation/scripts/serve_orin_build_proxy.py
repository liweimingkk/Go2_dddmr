#!/usr/bin/env python3
"""Small allow-listed HTTP/CONNECT proxy for an offline Orin Docker build."""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import ipaddress
import sys
from typing import List, Optional, Tuple
from urllib.parse import urlsplit


MAX_HEADER_BYTES = 64 * 1024
ALLOWED_TARGET_PORTS = {80, 443}


def parse_authority(authority: str, default_port: int) -> Tuple[str, int]:
    authority = authority.strip()
    if not authority:
        raise ValueError("empty target authority")
    if authority.startswith("["):
        close = authority.find("]")
        if close < 0:
            raise ValueError("invalid IPv6 authority")
        host = authority[1:close]
        suffix = authority[close + 1 :]
        port = int(suffix[1:]) if suffix.startswith(":") else default_port
        return host, port
    if authority.count(":") == 1:
        host, port_text = authority.rsplit(":", 1)
        return host, int(port_text)
    return authority, default_port


async def pump(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
    try:
        while True:
            data = await reader.read(64 * 1024)
            if not data:
                break
            writer.write(data)
            await writer.drain()
    except (ConnectionError, asyncio.CancelledError):
        pass
    finally:
        with contextlib.suppress(Exception):
            writer.write_eof()


async def relay(
    client_reader: asyncio.StreamReader,
    client_writer: asyncio.StreamWriter,
    upstream_reader: asyncio.StreamReader,
    upstream_writer: asyncio.StreamWriter,
) -> None:
    tasks = {
        asyncio.create_task(pump(client_reader, upstream_writer)),
        asyncio.create_task(pump(upstream_reader, client_writer)),
    }
    done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    for task in pending:
        task.cancel()
    for task in done | pending:
        with contextlib.suppress(asyncio.CancelledError, ConnectionError):
            await task


def filtered_request(
    method: str,
    target: str,
    version: str,
    header_lines: List[str],
) -> Tuple[str, int, bytes]:
    parsed = urlsplit(target)
    if parsed.scheme.lower() != "http" or not parsed.hostname:
        raise ValueError("regular proxy requests must use an absolute http:// URL")
    host = parsed.hostname
    port = parsed.port or 80
    if port not in ALLOWED_TARGET_PORTS:
        raise ValueError(f"target port {port} is not allowed")
    origin_target = parsed.path or "/"
    if parsed.query:
        origin_target += f"?{parsed.query}"

    retained = []
    saw_host = False
    for line in header_lines:
        name = line.partition(":")[0].strip().lower()
        if name in {"connection", "proxy-connection", "proxy-authorization"}:
            continue
        if name == "host":
            saw_host = True
        retained.append(line)
    if not saw_host:
        retained.append(f"Host: {parsed.netloc}")
    retained.append("Connection: close")
    encoded = (
        f"{method} {origin_target} {version}\r\n"
        + "\r\n".join(retained)
        + "\r\n\r\n"
    ).encode("iso-8859-1")
    return host, port, encoded


class BuildProxy:
    def __init__(self, allowed_client: str, verbose: bool) -> None:
        self.allowed_client = ipaddress.ip_address(allowed_client)
        self.verbose = verbose

    def log(self, message: str) -> None:
        print(message, file=sys.stderr, flush=True)

    async def reject(
        self, writer: asyncio.StreamWriter, status: str, message: str
    ) -> None:
        payload = message.encode("utf-8")
        writer.write(
            (
                f"HTTP/1.1 {status}\r\n"
                f"Content-Length: {len(payload)}\r\n"
                "Connection: close\r\n\r\n"
            ).encode("ascii")
            + payload
        )
        with contextlib.suppress(ConnectionError):
            await writer.drain()

    async def handle(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        peer = writer.get_extra_info("peername")
        peer_text = str(peer[0]) if peer else ""
        try:
            peer_ip = ipaddress.ip_address(peer_text)
        except ValueError:
            peer_ip = None
        if peer_ip != self.allowed_client:
            self.log(f"BUILD_PROXY_REJECTED_CLIENT={peer_text}")
            await self.reject(writer, "403 Forbidden", "client is not allowed")
            writer.close()
            await writer.wait_closed()
            return

        upstream_writer: Optional[asyncio.StreamWriter] = None
        try:
            raw_header = await reader.readuntil(b"\r\n\r\n")
            if len(raw_header) > MAX_HEADER_BYTES:
                raise ValueError("request header is too large")
            decoded = raw_header.decode("iso-8859-1")
            lines = decoded[:-4].split("\r\n")
            method, target, version = lines[0].split(" ", 2)
            method = method.upper()

            if method == "CONNECT":
                host, port = parse_authority(target, 443)
                if port not in ALLOWED_TARGET_PORTS:
                    raise ValueError(f"target port {port} is not allowed")
                upstream_reader, upstream_writer = await asyncio.open_connection(
                    host, port
                )
                writer.write(b"HTTP/1.1 200 Connection Established\r\n\r\n")
                await writer.drain()
            else:
                host, port, request = filtered_request(
                    method, target, version, lines[1:]
                )
                upstream_reader, upstream_writer = await asyncio.open_connection(
                    host, port
                )
                upstream_writer.write(request)
                await upstream_writer.drain()

            if self.verbose:
                self.log(f"BUILD_PROXY_REQUEST={peer_text} {method} {host}:{port}")
            await relay(reader, writer, upstream_reader, upstream_writer)
        except (asyncio.IncompleteReadError, ConnectionError):
            pass
        except Exception as exc:
            self.log(f"BUILD_PROXY_ERROR={peer_text} {type(exc).__name__}: {exc}")
            if not writer.is_closing():
                await self.reject(writer, "502 Bad Gateway", str(exc))
        finally:
            if upstream_writer is not None:
                upstream_writer.close()
                with contextlib.suppress(Exception):
                    await upstream_writer.wait_closed()
            writer.close()
            with contextlib.suppress(Exception):
                await writer.wait_closed()


async def async_main(args: argparse.Namespace) -> None:
    proxy = BuildProxy(args.allow_client, args.verbose)
    server = await asyncio.start_server(
        proxy.handle,
        args.bind,
        args.port,
        limit=MAX_HEADER_BYTES,
    )
    sockets = ", ".join(str(sock.getsockname()) for sock in server.sockets or [])
    proxy.log(f"BUILD_PROXY_LISTEN={sockets}")
    proxy.log(f"BUILD_PROXY_ALLOWED_CLIENT={args.allow_client}")
    async with server:
        await server.serve_forever()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Allow-listed transient HTTP proxy for the Orin build"
    )
    parser.add_argument("--bind", default="192.168.123.222")
    parser.add_argument("--port", type=int, default=3128)
    parser.add_argument("--allow-client", default="192.168.123.18")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    try:
        asyncio.run(async_main(args))
    except KeyboardInterrupt:
        return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
