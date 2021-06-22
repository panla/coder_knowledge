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

set t_Co=256

set showmatch
set tabstop=4
set expandtab
set shiftwidth=4
set autoindent
set backspace=indent,eol,start

set paste
set listchars=tab:>-,trail:-

if has( 'mouse' )
    set mouse-=a
endif
```

`/etc/vim/vimrc`

```text
syntax on

set showcmd
set showmatch
set smartcase
set incsearch
set mouse=a
```
