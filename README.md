# Python 项目模板

用于创建个人 Python 脚本和中小型 DIY 项目。复制本目录后，将文件夹重命名为实际项目名称。

## 目录说明

- `src/`：Python 源代码；默认入口为 `src/main.py`。
- `config/config.example.yaml`：可公开的配置示例。
- `config/config.local.yaml`：本机配置，不提交。
- `data/example/`：匿名示例数据。
- `data/private/`：真实或私人数据，不提交。
- `docs/`：设计、记录、待办和理解文档。
- `tests/`：测试代码。
- `build/`、`output/`：构建文件和运行输出，不提交。

## 运行

在项目根目录使用 CMD 或 PowerShell 执行：

```cmd
python src\main.py
```

## Git 与公开导出

模板目录本身不初始化 Git。复制为实际项目后再初始化仓库。
.gitignore 用于排除本机配置、私人数据和生成文件；`.gitattributes` 中的 `export-ignore` 仅在使用 `git archive` 导出时生效。

项目需要安装、被其他项目导入或作为库发布时，再改为正式包结构并添加 `pyproject.toml`。准备授权他人使用时，再选择并添加 `LICENSE`。
