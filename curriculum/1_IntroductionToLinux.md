# Introduction to linux

## Installation

### Linux installation

- USB drive
- Dual boot
- second laptop

detailed instruction can be found [here](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)

detailed instructions for dual boot can be found [here](https://medium.com/linuxforeveryone/how-to-install-ubuntu-20-04-and-dual-boot-alongside-windows-10-323a85271a73)

#### drive partitioning

If you are manually partitioning the drive, create the following partitions:

- **root partition** with ext4 filesystem and / mount point
- **swap partition** for recommended size of swap partition see table following [page](https://help.ubuntu.com/community/SwapFaq)

### Docker installation

[Moveit and Docker best practices](https://picknik.ai/ros/robotics/docker/2021/07/20/Vatan-Aksoy-Tezer-Docker.html)

Follow the instructions from the docker [documentation](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)

Follow the first chapter _Manage Docker as a non root user_ of the linux post installation [instructions](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)

If you have an Nvidia graphics card and want to enable its use inside the container follow the nvidia [docs](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit). You should check that the Nvidia driver is running correctly  first with the command ```nvidia-smi``` prior to installing the Nvidia container toolkit.

Test your docker installation with the command ```docker run hello-world``` or with Nvidia support ```docker run --rm --gpus all nvidia/cuda:11.0.3-base-ubuntu20.04 nvidia-smi```

### Additional software

required:

- git ```sudo apt install git```
- [vscode](https://code.visualstudio.com/download) download the .deb file and open with software installer
- vscode extensions
  - [ROS](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros)
  - [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python)
  - [Remote development](https://code.visualstudio.com/docs/remote/remote-overview)
  - [docker](https://code.visualstudio.com/docs/containers/overview)
  - [github copilot](https://marketplace.visualstudio.com/items?itemName=GitHub.copilot)

optional:

- [gitkraken](https://www.gitkraken.com/) (if you desire GUI environment for git)

&nbsp;

## Bash fundamentals (CLI)

&nbsp;

### Copy Paste

You can copy from and past to the command line with:  

_Ctrl-Shift-C_ and _Ctrl-Shift-V_  
&nbsp;

### Structure of commands

The full syntax for a Bash command is:  

```bash
command [options] [arguments]
```

Bash treats the first string it encounters as a command. The following command uses Bash's ls (for "list") command to display the contents of the current working directory:

```bash
ls
```

Most Bash commands have options for modifying how they work. Options, also called flags, give a command more specific instructions. As an example, files and directories whose names begin with a period are hidden from the user and are not displayed by ls. However, you can include the -a (for "all") flag in an ls command and see everything in the target directory:

```bash
ls -a /etc
```

You can even combine flags for brevity. For example, rather than enter ls -a -l /etc to show all files and directories in Linux's /etc directory in long form, you can enter this instead

```bash
ls -al /etc
```

&nbsp;

### Get help

Which options and arguments can be used, or must be used, varies from command to command. Fortunately, Bash documentation is built into the operating system. Help is never more than a command away. To learn about the options for a command, use the man (for "manual") command. For instance, to see all the options for the mkdir ("make directory") command, do this:  

```bash
man ls
```

Most Bash and Linux commands support the --help option. This shows a description of the command's syntax and options. To demonstrate, enter mkdir --help. The output will look something like this:

```bash
ls --help
```

&nbsp;

### Autocomplete

Let's say you want to read the contents of one of the directories that you just found.
To use this command

- you could use the full file name, such as:

```bash
ls Documents
```

- Instead, you can use Bash's rudimentary autocompletion to do most of the work for you. Try typing:

```bash
ls Doc
```

Then press __Tab__  
&nbsp;

### Working directory

One important concept to understand is that the shell has a notion of a default location in which any file operations will take place. This is its working directory. If you try to create new files or directories, view existing files, or even delete them, the shell will assume you’re looking for them in the current working directory unless you take steps to specify otherwise. So it’s quite important to keep an idea of what directory the shell is “in” at any given time. The `pwd` command will tell you exactly what the current working directory is.

```bash
pwd
```

You can change the working directory using the cd command, an abbreviation for ‘change directory’. Try typing the following:

```bash
cd /
pwd
```

&nbsp;

### Relative and absolute paths

Most of the examples we’ve looked at so far use relative paths. That is, the place you end up at depends on your current working directory. Consider trying to cd into the “etc” folder. If you’re already in the root directory that will work fine:

```bash
cd /
cd etc
```

But what if you’re in your home directory?
You’ll see an error saying “No such file or directory”. But we have seen two commands that are absolute. No matter what your current working directory is, they’ll have the same effect. The first is when you run `cd` on its own to go straight to your home directory. The second is when you used `cd /` to switch to the root directory. In fact any path that starts with a forward slash is an absolute path. You can think of it as saying “switch to the root directory, then follow the route from there”. That gives us a much easier way to switch to the etc directory, no matter where we currently are in the file system:

```bash
cd /etc
```

It also gives us another way to get back to your home directory, and even to the folders within it. Suppose you want to go straight to your “Desktop” folder from anywhere on the disk (note the upper-case “D”). In the following command you’ll need to replace USERNAME with your own username, the `whoami` command will remind you of your username, in case you’re not sure:

```bash
whoami
cd /home/USERNAME/Desktop
pwd
```

There’s one other handy shortcut which works as an absolute path. As you’ve seen, using “/” at the start of your path means “starting from the root directory”. Using the tilde character (”~”) at the start of your path similarly means “starting from my home directory”.

```bash
cd ~
```

&nbsp;

## Bash commands

&nbsp;

### ls command

`ls` lists the contents of your current directory or the directory specified in an argument to the command. By itself, it lists the files and directories in the current directory:

```bash
ls
```

Files and directories whose names begin with a period are hidden by default. To include these items in a directory listing, use an -a flag:

```bash
ls -a
```

To get even more information about the files and directories in the current directory, use an -l flag:

```bash
ls -l
```

&nbsp;

### cd command

`cd` stands for "change directory," and it does exactly what the name suggests: it changes the current directory to another directory. It enables you to move from one directory to another just like its counterpart in Windows. The following command changes to a subdirectory of the current directory named orders:

```bash
cd orders
```

You can move up a directory by specifying `..`as the directory name:

```bash
cd ..
````

You can also use .. more than once if you have to move up through multiple levels of parent directories:  

```bash
cd ../..
```

This command changes to your home directory—the one that you land in when you first log in:

```bash
cd ~
```

&nbsp;

### mkdir command

You can create directories by using the `mkdir` command. The following command creates a subdirectory named orders in the current working directory:

```bash
mkdir orders
```

If you want to create a subdirectory and another subdirectory under it with one command, use the -p flag:

```bash
mkdir -p orders/2019
```

&nbsp;

### rm command

The `rm` command is short for "remove." As you'd expect, rm deletes files. So this command puts an end to 0001.jpg:

```bash
rm 0001.jpg
```

Be wary of `rm`. The dreaded `rm -rf /` command deletes every file on an entire drive. It works by recursively deleting all the subdirectories of root and their subdirectories. The -f (for "force") flag compounds the problem by suppressing prompts. Don't do this. If you want to delete a subdirectory named orders that isn't empty, you can use the rm command this way:

```bash
rm -r orders
```

&nbsp;

### cp command

The `cp` command copies not just files, but entire directories (and subdirectories) if you want. To make a copy of __0001.jpg__ named __0002.jpg__, use this command

```bash
cp 0001.jpg 0002.jpg
```

If 0002.jpg already exists, Bash silently replaces it. That's great if it's what you intended, but not so wonderful if you didn't realize you were about to overwrite the old version.

### mv command

Move a file to another folder.

```bash
mv source_path destination_path
```

&nbsp;

## __File permissions__

Every file and directory on your Unix/Linux system is assigned 3 types of owner, given below.

- __u__ (user),the owner of a file is granted any of the permissions.  
- __g__ (group), group the file belongs to is granted a permission. A user- group can contain multiple users.
- __o__ (other), all others are granted a permission.

Every file and directory in your UNIX/Linux system has following 3 permissions defined for all the 3 owners discussed above.  

- __r__ (read) file/directory may be opened for read access.  
- __w__ (write) file/directory may be opened for write/edit access.  
- __x__ (execute) file may be executed as a program/directory may be traversed.

Let’s see file permissions in Linux with examples:

```bash
ls - l
```

returns

`drwxr-xr-x 2 username username     4096 Aug  9 13:39  Documents`  
`-rw-r--r-- 1 username username     3771 Apr 17  2021  .bashrc`  

The first character

- `d` directory
- `-` file

Following characters

- __r__ = read permission  
- __w__ = write permission  
- __x__ = execute permission  
- __–__ = no permission

Format

- `rwxrwxrwx user group`

By design, many Linux distributions will add users to a group of the same group name as the user name. Thus, a user ‘username’ is added to a group named ‘username’.

For the Directory Documents  
`user`  
user 'username' has read, write, and execute permission  
`group`  
user-group 'username' has read and execute permission  
`other`  
has execute permission  
&nbsp;

### The superuser

The superuser is, as the name suggests, a user with super powers. As for those super powers: root can modify or delete any file in any directory on the system, regardless of who owns them; root can rewrite firewall rules or start network services that could potentially open the machine up to an attack; root can shutdown the machine even if other people are still using it. In short, root can do just about anything, skipping easily round the safeguards that are usually put in place to stop users from overstepping their bounds.  

`sudo` is used to prefix a command that has to be run with superuser privileges. A configuration file is used to define which users can use sudo, and which commands they can run. When running a command like this, the user is prompted for their own password, which is then cached for a period of time (defaulting to 15 minutes), so if they need to run multiple superuser-level commands they don’t keep getting continually asked to type it in.  
&nbsp;

### Creating files and changing permissions

The touch command allows you to create a file, for example

```bash
touch my_script.py
```

will create a new file named 'my_script' with the '.py' python extension  
&nbsp;

[vscode](https://code.visualstudio.com/) is the python IDE we will using, you can open files and directories from the cli using the `code` command.

```bash
code my_script.py
````

add the following to the python file

```python
#!/usr/bin/env python3

print('hello world')
```

Close and save the file.  
Now try run the file

```bash
./my_script.py
```

will result in the following error:  
`bash: ./my_scripts.py: Permission denied`

If we inspect the permissions of our newly created file using the `ls -l` command

`-rw-rw-r-- 1 username username  my_script.py`  

In order to able to execute the python file we need to give the user 'username' execute permission. This can be done with the `chmod` command. `u+x` will add execute permission for the user.

```bash
chmod u+x my_script.py
```

will result in:  
`-rwxrw-r-- 1 username username  test.py`

```bash
chmod +x my_script.py
```

will result in:  
`-rwxrwxr-x 1 username username  test.py`  
now try to run the file again

```bash
./my_script.py
```

you should see `hello world`  
&nbsp;

## Docker commands

Familiarize yourself with the following Docker CLI commands

- [docker build](https://docs.docker.com/engine/reference/commandline/build/)
- [docker run](https://docs.docker.com/engine/reference/commandline/run/)
- [docker image ls](https://docs.docker.com/engine/reference/commandline/image_ls/)
- [docker image prune](https://docs.docker.com/engine/reference/commandline/image_prune/)
- [docker container list](https://docs.docker.com/engine/reference/commandline/container_ls/)
- [docker container prune](https://docs.docker.com/engine/reference/commandline/container_prune/)

## Configure git

Configure setup and configure git on your machine.

setup your identity, [name](https://docs.github.com/en/get-started/getting-started-with-git/setting-your-username-in-git) and [email address](https://docs.github.com/en/account-and-profile/setting-up-and-managing-your-personal-account-on-github/managing-email-preferences/setting-your-commit-email-address) it is recommended to use the github noreply email address otherwise you will expose your email address to the public.

```bash
git config --global user.name "John Doe"
git config --global user.email johndoe@users.noreply.github.com
```

[generate a new ssh key](https://docs.github.com/en/github/authenticating-to-github/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) and [add](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account) it to your github account.

## References

### Linux

- [The linux command line for beginners](https://ubuntu.com/tutorials/command-line-for-beginners#1-overview)
- [introduction to bash](https://learn.microsoft.com/en-gb/training/modules/bash-introduction/)
- [40 Essential Linux Commands That Every User Should Know](https://www.hostinger.com/tutorials/linux-commands#1_sudo_command)
- [How to use chmod to change file permissions](https://www.howtogeek.com/437958/htg-explains-what-are-linux-file-permissions/)
- [apt command in linux](https://linuxize.com/post/how-to-use-apt-command/)
