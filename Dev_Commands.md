
rm file.txt             # Delete a file
rm -r folder_name       # Delete a folder (recursive)
clear                   # Clear terminal
```

## 💻 VS Code with WSL
```
code .                  # Open current folder in VS Code (with Remote-WSL)
```

## 📦 Installing Packages (Ubuntu)
```
sudo apt update         # Update package list
sudo apt install name   # Install a package
```

## 🔧 Git Setup (One-time)
```
git config --global user.name "Your Name"
git config --global user.email "your@email.com"
```

## 📂 Git Basics
```
git init                        # Create new git repo in current folder
git clone <url>                  # Download repo from GitHub
git status                       # Check repo status
git add file.txt                  # Stage a file
git add .                         # Stage all changes
git commit -m "message"           # Commit changes
git push                          # Push to remote repo
git pull                          # Pull latest from remote
```

## 🌿 Branching
```
git branch                        # List branches
git checkout -b new_branch        # Create + switch to new branch
git checkout branch_name          # Switch branch
git merge branch_name             # Merge branch into current
```

## 🚑 Common Fixes
```
git pull --rebase origin main     # Sync local with remote before pushing
git reset --hard HEAD~1           # Undo last commit (permanent)
git rm --cached file              # Untrack a file without deleting it
```
