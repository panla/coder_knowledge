# install vscode on linux

## 下载安装

[下载地址](https://code.visualstudio.com/Download)

把下载链接的 `https://az764295.vo.msecnd.net` 替换为 `http://vscode.cdn.azure.cn`

[最新下载cdn](http://vscode.cdn.azure.cn/stable/c3f126316369cd610563c75b1b1725e0679adfb3/code-stable-x64-1626303100.tar.gz)

解压

Desktop 文件

`/usr/share/applications/code.desktop`

```text
[Desktop Entry]
Name=Visual Studio Code
Comment=Code Editing. Redefined.
GenericName=Text Editor
Exec=/home/opt/VSCode/bin/code --no-sandbox --unity-launch %F
Icon=/home/opt/VSCode/resources/app/resources/linux/code.png
Type=Application
StartupNotify=false
StartupWMClass=Code
Categories=Utility;TextEditor;Development;IDE;
MimeType=text/plain;inode/directory;
Actions=new-empty-window;
Keywords=vscode;

[Desktop Action new-empty-window]
Name=New Empty Window
Exec=/home/opt/VSCode/bin/code --no-sandbox --unity-launch %F
Icon=/home/opt/VSCode/resources/app/resources/linux/code.png
```

## 插件

```text
Bracket Pair Colorizer
indent-rainbow
Chinese Language Pack
Todo Tree
vscode-icons

Docker
Remote-Containers
Redis

Git Graph
Git History
gitignore
GitLens--Git supercharged

Markdown Preview Enhanced
markdownlint

MySQL Syntax
Prettify JSON

Jupyter
YAML
EditorConfig for VS Code

Pylance
Python
Python Indent
Python Type Hint

Better C++ Syntax
C/C++
C/C++ Snippets
C++ Intellisense

Debugger for Java
Language Support for Java
Maven for java
Project Manager for java
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
