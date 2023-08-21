# Lab Guide
A comprehensive lab guide detailing how to use ROS, the Sawyer robots, the Turtlebots and other useful information for 106 lab.
# Setup
The Lab Guide uses Just the Docs, a documentation theme for Jekyll, that builds pages using Markdown. The template for Just the Docs can be found [here](https://github.com/just-the-docs/just-the-docs-template). To run the Lab Guide locally so you can changes before commiting them, you will need to install [Ruby](https://rubyinstaller.org/downloads/) and Jekyll. Once you have Ruby installed, use gem (the Ruby package manager) to install Jekyll and Bundler:
```
  gem install bundler jekyll
  bundle install
```
# Running Locally
To run the Lab Guide locally, navigate to the Lab Guide directory and run:
```
  bundle exec jekyll serve
```
Now you can view the Lab Guide at http://localhost:4000/. Any changes you make to the Lab Guide will be reflected in the local version. To stop the local server, press `Ctrl + C` in the terminal.