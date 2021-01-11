# Git Good.
##### An intro guide to Git
by Geoffrey Xue
#
Quick tips:
- If you haven't work with terminal before, ```dir``` will show you the folders (the ones labelled <DIR>) and files in your current location, and ```cd folderName``` will transfer you to that folder. ```cd ..``` to go backwards.
- To work on a project on Github or Gitlab, you must first have Git itself. Head [**here**](https://git-scm.com/downloads) to download Git. You can check if you have it already by opening terminal and typing in ```git```
- ```git branch``` to check current branch and other local branches
- ```git status``` to check any changes, staged changed, commits on the current branch

### How does Git work?
If have a project, you'll typically need a repository (repo) on Github or Gitlab. 
A repo allows you to: 
- Create variations of your current work (called branches)
- Monitor your history of changes
- Safeguards your project from accidental unrecoverable deletes
- And more!
#
Each repository has **branches**, and will start with one called **master**. You can create new ones, protect them, delete them, and merge them. Each branch will have a saved instance of your work, and you can work on and update them as you like.

Git works as an intermediary manager between your local work and the work saved on Github. Here's a breakdown:

You can establish a connection with your Github repo by setting a **remote URL**, typically called **origin**

You can **push** and **pull** code to and from your repository set up in your **origin**
You can retrieve updates with **fetch** from your repository set up in your **origin**

You can **checkout** different branches and **push** or **pull** code from there

After you've saved the changes on your files themselves, you can **add** them, which **stages** the changes
Once you've **add**ed changes, you can **commit** them with a message
Finally, after you've **commit**ted your changes, you can **push** them

In terminal, these all translate to:
```bash
git remote add origin http://github.com/user/repo.git

git push origin branch
git pull origin branch
git fetch origin 

git checkout branch

git add .
git commit -am "message"
```
## Push to a new repo with a project you're working on:
1. Go to Github
2. Create a new repo and name your repository something (this will be the default folder name of any future cloned versions)
![Create repo](https://i.ibb.co/N3zddQT/Create-Repo.png)
3. Copy the remote URL in Quick setup
![Copy setup](https://i.ibb.co/sb85vbw/Copy-Origin.png)
4. Go to terminal and enter these procedurally
```bash
cd yourProject
git init
git remote add origin https://github.com/user/repo.git
git config --global user.email "yourgithubemail@gmail.com"
```
5. You should now see a verification screen. Authorize the Git Credentials Manager
###### If you don't see a verification screen, you can also just specify your password with ```git config --global user.password "yourPassword"```
6. Back to your terminal that's currently in your project folder
```bash
git add .
git commit -am "initial commit"
git push origin master
```
7. You should now see something like this in your terminal, as well as a project in your repository
```bash
To https://github.com/user/repo.git * [new branch]
master -> master
```
###### If these steps didn't work, the next-most probable issue is that you don't have access to the repository
#
#
## Clone a project
1. Open up terminal
```bash
cd yourDevelopmentFolderWithOtherProjects
```
###### *Git will create a folder named after the repository for you and shove all the code inside that folder, so you don't need to create a specific folder for a git project. Otherwise, you'll end up with a double-foldered project (Ex: Projects/myProject/myProject/(code))
#
```bash
git clone https://github.com/user/repo.git
```
#
#
## Create a new branch locally and then push it:
Requirements: A project with git and a connected remote URL (a repository on github that you've pushed to)
The -b tag on checkout indicates you're creating a new branch
1. Open up terminal
```bash
cd yourProject
git checkout -b yourBranchName
git push origin yourBranchName
```
2. If you have uncommitted changes, ```git add .``` and ```git commit -am "message"``` before pushing
#
#
## Switch to a branch that exists:
Requirements: A project with git and a connected remote URL (a repository on github that you've pushed to)
###### Make sure you don't have any uncommited changes. If you do, push them first. If you don't want them, run ```git reset --hard HEAD```
1. Open up terminal
```bash
git branch checkout existingBranchName
git pull origin existingBranchName
```
#
#
## Setting up a .gitignore
A .gitignore prevents some specified files and folder from being sent into your repository. If there are extremely large files (hundreds of MB or GB), you probably should not commit them to not hit your 10 GB repository limit.
Requirements: A project with git and a connected remote URL (a repository on github that you've pushed to)
1. Create a new file in your folder named ".gitignore"
Add these into your .gitignore file depending on your needs:
```bash
ignoredFolder/
ignoredFile.fileType
*.ignoredFileType
```
###### For more examples, check out this [tutorial](https://www.atlassian.com/git/tutorials/saving-changes/gitignore)
2. Commit your changes
```bash
git add .
git commit -am "message"
```
2. Make your gitignore take effect and push to your repository
```bash
git rm -r --cached .
git add .
git commit -m "fixed untracked files"
git push origin branchName
```








