#!/usr/bin/env python3
"""Pure helpers shared by the live Go2/XT16 clock-offset samplers."""

from __future__ import annotations

from collections.abc import Sequence


def ordered_pair_indices(
    target_receipts_ns: Sequence[int],
    source_receipts_ns: Sequence[int],
    window_ns: int,
) -> list[tuple[int, int]]:
    """Return an ordered maximum-cardinality, minimum-cost receipt matching.

    Each input sample is consumed at most once. Among matchings with the same
    number of pairs, the dynamic program minimizes total absolute receipt
    distance. Inputs must be sorted in ascending receipt order.
    """
    if window_ns < 0:
        raise ValueError("window_ns must be nonnegative")
    if any(a > b for a, b in zip(target_receipts_ns, target_receipts_ns[1:])):
        raise ValueError("target receipts must be sorted")
    if any(a > b for a, b in zip(source_receipts_ns, source_receipts_ns[1:])):
        raise ValueError("source receipts must be sorted")

    target_count = len(target_receipts_ns)
    source_count = len(source_receipts_ns)
    if target_count == 0 or source_count == 0:
        return []

    # Actions: 1=skip target, 2=skip source, 3=pair. Scores need only two
    # rows; the byte action table is retained for path reconstruction.
    actions = bytearray(target_count * source_count)
    next_matches = [0] * (source_count + 1)
    next_cost = [0] * (source_count + 1)

    for target_index in range(target_count - 1, -1, -1):
        current_matches = [0] * (source_count + 1)
        current_cost = [0] * (source_count + 1)
        for source_index in range(source_count - 1, -1, -1):
            best_matches = next_matches[source_index]
            best_cost = next_cost[source_index]
            best_action = 1
            best_priority = 0

            candidates = [
                (
                    current_matches[source_index + 1],
                    current_cost[source_index + 1],
                    2,
                    1,
                )
            ]
            receipt_delta_ns = (
                target_receipts_ns[target_index]
                - source_receipts_ns[source_index]
            )
            if abs(receipt_delta_ns) <= window_ns:
                candidates.append(
                    (
                        1 + next_matches[source_index + 1],
                        abs(receipt_delta_ns) + next_cost[source_index + 1],
                        3,
                        2,
                    )
                )

            for matches, cost, action, priority in candidates:
                if (
                    matches > best_matches
                    or (matches == best_matches and cost < best_cost)
                    or (
                        matches == best_matches
                        and cost == best_cost
                        and priority > best_priority
                    )
                ):
                    best_matches = matches
                    best_cost = cost
                    best_action = action
                    best_priority = priority

            current_matches[source_index] = best_matches
            current_cost[source_index] = best_cost
            actions[target_index * source_count + source_index] = best_action

        next_matches = current_matches
        next_cost = current_cost

    pairs: list[tuple[int, int]] = []
    target_index = 0
    source_index = 0
    while target_index < target_count and source_index < source_count:
        action = actions[target_index * source_count + source_index]
        if action == 3:
            pairs.append((target_index, source_index))
            target_index += 1
            source_index += 1
        elif action == 2:
            source_index += 1
        else:
            target_index += 1
    return pairs


def clock_phase_diagnostics(
    receipts_ns: Sequence[int], header_stamps_ns: Sequence[int]
) -> tuple[int, int]:
    """Return maximum adjacent phase step and full-window phase span."""
    if len(receipts_ns) != len(header_stamps_ns):
        raise ValueError("receipt and header sequences must have equal length")
    phases = [
        header_stamp_ns - receipt_ns
        for receipt_ns, header_stamp_ns in zip(receipts_ns, header_stamps_ns)
    ]
    if not phases:
        return 0, 0
    max_step_ns = max(
        (abs(current - previous) for previous, current in zip(phases, phases[1:])),
        default=0,
    )
    return max_step_ns, max(phases) - min(phases)
