# Go2_dddmr

## 版本控制工作流

仓库根目录的 `AGENTS.md` 规定了修改、校验、提交和推送要求。完成一组相关修改后，先运行基础检查：

```bash
./scripts/check-repository.sh
```

确认修改正确后，在功能分支上显式列出需要提交的文件：

```bash
./scripts/submit-change.sh "说明本次修改的目的" -- path/to/file another/file
```

该脚本不会执行 `git add -A`，也不会直接提交到 `main` 或 `master`。它只暂存给出的路径，检查常见提交问题，创建提交并推送当前分支。GitHub Actions 会在拉取请求和 `main` 分支更新时重复运行相同的基础检查。
