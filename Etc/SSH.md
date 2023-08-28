---
title: "SSH"
nav_order: 3
parent: "Miscellaneous"
layout: default
has_toc: false
has_children: false
---

# Setting Up an SSH key for Github
As you might have noticed, the functionality to login to github from Github CLI has been depricated. It's sad :,> but worry not, this section here will teach you how to push your code into private repository using Github SSH keys. Think of this as pushing your code with some secret key that Github can use to determine your Github User ID, hence granting you access to modify code on the Github repositories you have access to. Let's get started!


## Steps:
1. Open a terminal if you have not yet done so. You can do so with the shortcut `CTRL+ALT+t`. Create a `~/.ssh/<YOUR_NAME>/` directory. This directory will be  used to store your super secret key!  
    ```
    mkdir -p ~/.ssh/<YOUR_NAME>/
    ```
2. Get the directory filepath.
    ```
    cd ~/.ssh/<YOUR_NAME>/
    realpath .
    ```
    This will return the absolute path to your key directory. We'll call this `<KEY_PATH>`. Copy this somewhere.
3. Create the key.   
    1. From any file location in the terminal, run this:
        ```
        ssh-keygen -t ed25519
        ```
        The command in the Lab1 doc also works, both commands are almost synonymous
    1. You will be prompted to enter the file in which the key will be saved. Enter the string that looks something like this: `"<KEY_PATH>" + "/id"`.  
        Your keypath string should look something like this:
        ```
        /home/cc/ee106a/fa23/class/ee106a-xxx/.ssh/chris/id
        ```
    1. Once you press "Enter" from the previous step, you will be prompted again. This is an optional password for you (the user) to key in whenever you want to use this secret key. This is useful when you're working on a computer with multiple users using the same account. The 106a lab does fall under this category of use-case. If you decide to use this, key in your password, and then re-confirm it. If you failed to re-confirm the password, don't worry. Just restart the steps to create a new key and there will be no issues.

    Congrats, You've just generated your random secret key!
4. Associate the key with your github account.  
    1. Now, open a browser and go to your Github Profile.
    1. Look for the Settings tab.  
        ![Github Settings Tab](/assets/images/Etc/SSH/instruction.png)
    1. Click into the "SSH and GPG Keys" tab.  
        ![Github ssh keys tab](/assets/images/Etc/SSH/instruction2.png)
    1. Click the light-green "New SSH key" button.  
        ![Creating a new ssh key](/assets/images/Etc/SSH/instruction3.png)
    1. Here is where you will key in the corresponding 
    access key. The "Title" field is arbitary, for you to be able identify this specific ssh key (for CS161/CS70 folks, it is like the RSA public key).  
        1. Get the public key by entering this:
            ```
            cat "<KEY_PATH>" + "/id.pub"
            ```
            The command should look something like this:
            ```
            cat /home/cc/ee106a/fa23/staff/ee106a-tad/.ssh/chris/id.pub
            ```
            Copy the outputs after running the command, and paste it into the "Key" field.
        1. Create the key.

1. Step 5 is what you have to run every time you want to `git push/pull/clone` from a Github repository that you have access to. It is recommended that you make this step as a `.bashrc` alias. Refer to the docs on what an alias is and how to do so or just ChatGPT. 
Start an SSH agent and adding your ssh key:  
    ```
    eval "($ssh-agent -s)"
    ssh-add ~/.ssh/<YOUR_NAME>/id
    ```
1. Verify your connection.  
    ```
    ssh -T git@github.com
    ```
    You should see a message like this:
    ```
    Hi <YOUR_USERNAME>! You’ve successfully authenticated, but GitHub does not provide shell access.
    ```
    


