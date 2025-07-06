apt update

# locality
apt install -y locales 
locale-gen en_US en_US.UTF-8 
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 
export LANG=en_US.UTF-8 
locale

# base tools
apt install -y curl git wget
git config --global core.autocrlf input

apt install -y vim
