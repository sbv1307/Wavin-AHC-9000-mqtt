# Hardening Raspberry Pi 3 MODEL B+ for the Docker Traefik router project

## Change password for user pi

Change password for user pi until final hardening, where user pi will be removed.

## Create new user

```bash
sudo adduser mqttmgr
```

Password = standard "r"

## Add user to sudoers

```bash
sudo adduser mqttmgr sudo
```

Verify by issuing the command

```bash
sudo su - mqttmgr
```
Prompt will change to **mqttmgr@pi-mqtt:~ $***


## Add aliases for user mqttmgr

In directory `/home/mqttmgr` create a file called `.bash_aliases` with the following content:

```bash
alias g='grep --color'
alias l='ls -lF --color'
alias la='l -a'
alias ll='ls -al'
# sort by mod time (t), newest last (r)
alias ltt='l -tr'
alias ltf='l -tr --full-time'
# list and sort by creation time
alias lc='l -trc'
alias lcf='l -trc --full-time'
alias psg='ps -e x -o ppid -o pid -o pgid -o tty -o vsz -o rss -o etime -o cputime -o rgroup -o ni -o fname -o args | grep'
alias psgc='psg cache_dirs'
alias pst='ps -fja'
alias ptuniq='pstree -p | sed "s/[0-9]//g" | uniq'
alias df='df -h'
alias pmem='ps aux | awk "{sum +=\$6}; END {print sum}"'
# diff -ruNp dir1 dir2 | less 
alias mdiff='diff -ruNp'
```

## Future hardening:
- Install mail client to alert about unattended upgrades
- Install `ufw` firewall

See [Security Setup & Hardening of Raspberry Pi OS](https://www.arch13.com/security-setup-hardening-of-raspberry-pi-os-buster/#Create-a-new-user-for-regular-use-and-maintenance) and [Raspberry Pi Hardening Guide](https://chrisapproved.com/blog/raspberry-pi-hardening.html) for more inspiration.
