生成 Doxygen 文档与 UML 的说明

前提：
- 已安装 doxygen（>=1.8）和 graphviz（dot）用于 class 图。
- 可选：安装 PlantUML 与 Java 来渲染 .puml 文件。

步骤：
1. 在项目根（本 README 所在目录的上级）运行：

```bash
# 进入 auto_aim 目录
cd /path/to/auto_aim
# 运行 doxygen，使用仓库根的 Doxyfile
doxygen Doxyfile
```

2. 结果输出在 `docs/doxygen/html`，打开 `index.html` 即可查看 API 文档和类图。

3. 若要使用 PlantUML 渲染项目内的 .puml 文件：

```bash
# 使用 plantuml 渲染到 png
plantuml docs/diagrams/auto_aim_classes.puml
# 生成的图片位于同目录，如 auto_aim_classes.png
```

常见问题：
- 如果 doxygen 没有生成类关系图，确保 graphviz 的 `dot` 在 PATH 中并且 `HAVE_DOT = YES`。
- 若使用 PlantUML，确保 Java 可用并且 plantuml.jar 可执行。
