最终交付脚本待当前事务完成后替换为96d6；final-install-stopped.log仍89d4中间版，其部署生命周期与最终版一致。

- 初始工作树维护版：`de60a4df4818fbaf477db95f4aec2b30ce9af93e8e58503586a5fb0704eeb593`，实际active旧版安装/重复/回退，以及首次inactive安装成功；日志 `actual-*`。
- 中间审查版：`89d4d2b439c63a59271ba072123051e3d166643d1f98e1abbb4a79d5933b9186`，实际inactive回退、再次inactive安装成功；日志 `final-install-stopped.log`、`final-rollback-stopped.log`。这些文件名包含final是当时标签，不等于最终交付SHA。
- 源码冻结提交 `41e1532e73d36005a80cd134807a70b0c0263675` 的最终交付版：`96d6b6f9b0005a859d856152631821cf0d4677bc1826fc2356a6318f45e75a5c`，日志统一 `release-final-*`，最终结果按这些日志分别列明。该版与89d4的现有unit安装生命周期代码相同；新增首装unit.d父路径symlink拒绝，且移动类说明串位置。
