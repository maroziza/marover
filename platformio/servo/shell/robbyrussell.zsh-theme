# prompt customization for zsh
PROMPT='%{$fg[green]%}ros %3~: %{$reset_color%}'
RPROMPT='%{$fg[white]%}$(git_prompt_info) %*%{$reset_color%}'

ZSH_THEME_GIT_PROMPT_PREFIX="|"
ZSH_THEME_GIT_PROMPT_SUFFIX="|"
ZSH_THEME_GIT_PROMPT_CLEAN="+"
