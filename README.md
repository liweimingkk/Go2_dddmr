# Go2_dddmr

## 版本控制工作流

当前采用两条长期分支：

- `main`：可部署、已经现场验收的稳定版。
- `develop`：下一个稳定版的集成分支。

短期分支使用 `feature/*`、`experiment/*`、`fix/*` 和 `release/*`。
`experiment/*` 不直接合入 `main`；先整理有效提交，再经
`feature/* -> develop -> release/* -> main -> tag` 晋级。

仓库根目录的 `AGENTS.md` 规定了修改、校验、提交和推送要求。完成一组相关修改后，先运行基础检查：

```bash
./scripts/check-repository.sh
```

确认修改正确后，在功能分支上显式列出需要提交的文件：

```bash
./scripts/submit-change.sh "说明本次修改的目的" -- path/to/file another/file
```

该脚本不会执行 `git add -A`，也不会直接提交到 `main`、`master`
或 `develop`。它只暂存给出的路径，检查常见提交问题，创建提交并推送当前分支。
GitHub Actions 会在拉取请求以及 `main`/`develop` 分支更新时重复运行相同的基础检查。

Go2 当前稳定导航基线和地图指纹见
[`dddmr_navigation/docs/go2_xt16_p2p_release_20260804.md`](dddmr_navigation/docs/go2_xt16_p2p_release_20260804.md)。

现场建图、地图保存、导航 dry-run 和真实导航的脚本命令见
[`dddmr_navigation/docs/go2_xt16_field_handover.md`](dddmr_navigation/docs/go2_xt16_field_handover.md)。
