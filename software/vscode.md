# install vscode on linux

## 下载安装

[下载地址](https://code.visualstudio.com/Download)

把下载链接的 `https://az764295.vo.msecnd.net` 替换为 `http://vscode.cdn.azure.cn`

[最新下载cdn](http://vscode.cdn.azure.cn/stable/b4c1bd0a9b03c749ea011b06c6d2676c8091a70c/code-stable-x64-1623259999.tar.gz)

解压

Desktop 文件

`/usr/share/applications/code.desktop`

```text
[Desktop Entry]
Name=Visual Studio Code
Comment=Code Editing. Redefined.
GenericName=Text Editor
Exec=执行文件绝对路径 --no-sandbox --unity-launch %F
Icon=com.visualstudio.code
# Icon 可以在解压后的文件里找到
Type=Application
StartupNotify=false
StartupWMClass=Code
Categories=Utility;TextEditor;Development;IDE;
MimeType=text/plain;inode/directory;
Actions=new-empty-window;
Keywords=vscode;

[Desktop Action new-empty-window]
Name=New Empty Window
Exec=执行文件绝对路径 --no-sandbox --unity-launch %F
Icon=com.visualstudio.code
# Icon 可以在解压后的文件里找到
```

## 插件

```text
Anaconda Extension Pack
Better C++ Syntax
C/C++
C/C++ Snippets
C++ Intellisense
Chinese Language Pack
EditorConfig for VS Code
Git Graph
Git History
gitignore
indent-rainbow
Jupyter
Markdown Preview Enhanced
markdownlint
MySQL Syntax
Prettify JSON
Pylance
Python
Python Indent
Python Type Hint
Rainbow CSV
YAML
```

## 字体与空格宽度

```text
Consolas, 'Courier New','monospace'
'Droid Sans Mono', 'monospace', monospace, 'Droid Sans Fallback'
```

## 错误

```text
error while loading shared libraries: libXss.so.1
```

```bash
dnf install libXScrnSaver
```
