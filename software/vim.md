# Vim

## 部分配置

`~/.vimrc`

```text
set fileencodings=utf-8,gb18030
set fileencoding=utf-8
set termencoding=utf-8
set encoding=utf-8

set nu

set cursorline

set mouse=a
set selection=exclusive
set selectmode=mouse,key

set showmatch
set tabstop=4
set shiftwidth=4
set autoindent

set paste
set listchars=tab:>-,trail:-

if has( 'mouse' )
    set mouse-=a
endif
```
