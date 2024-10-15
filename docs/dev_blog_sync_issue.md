---
layout: page
title: Dev Blog Sync Issue
description: Automatically updating my dev blog splash page
hide_description: true
date: 14 Oct 2024
---
I want to make it easier to write dev blog posts. I have created a markdown template and want my dev blog splash page to auto update when I add a new post.

The dev blog page splash page currently has this formatting:

```
## Personal Website

* [Website]{:.heading.flip-title} --- My Portfolio website

[website]: /docs/website.md

```

So Ideally I want want to automate the docs.md page. Here is a quick problem statement:
	
	I am writing a website in Jekyll I have a docs.md page that contains a series of blogs in markdown format. I want to somehow automatically update the docs.md page so it automatically updates when I have a new blog markdown file. This is the current format of my docs.md page
	
	```
	## Personal Website
	
	* [Website]{:.heading.flip-title} --- My Portfolio website
	
	[website]: /docs/website.md
	```
	
	

Ok so apparently I can use something called liquid templating. I can grab the **page** tag capture all my dev blog posts like this

```
{% raw %}
{% for post in site.pages %}
{% if post.path contains "docs/" and post.layout == "page" %}
* [{{ post.title }}]({{ post.url }}) - {{ post.description }}
{% endif %}
{% endfor %}
{% endraw %}
```


Testing it now but I also forget how to build the page. For future reference here is the command and its hosted at http://localhost:4000

```bash
bundle exec jekyll serve
```

The main thing is now in my blog posts I need to make sure the description is good because now it user facing. Easy enough fix for the future. 

Ok now I want to be able to sync my Obsidian vault to my website blog posts so I don't have to manually add them every time. I can use rsync for this. 

General syntax is 

```bash
rsync -av /path/to/folder1/ path/to/folder2/
```

For my specific case the syntax is 

```bash

rsync -av ~/brain/website/dev_blog/dev_blog_posts/ ~/Documents/Projects/liamjosephnolan.github.io/docs/

```

Originally I was using the --delete option and accidentally deleted all my blog posts. For now I will just push new files. Lets see if this creates an issues with duplicate files.


Ok it created the exact issue I just described. I want it to overwrite existing files so I need to use the update syntax indicated with -u 

So the final syntax is 

```bash
rsync -av -u ~/brain/website/dev_blog/dev_blog_posts/ ~/Documents/Projects/liamjosephnolan.github.io/docs/
```

I also added an alias in my .bashrc file with the following

```bash
alias sync_blog="rsync -av -u ~/brain/website/dev_blog/dev_blog_posts/ ~/Documents/Projects/liamjosephnolan.github.io/docs/"
```

Now everytime I run sync_blog in my terminal my blog posts should be updated. Seems to be working well enough. Next I want to add a date section in my blog posts for tracking timelines.

The date on other pages is contained in the title and has the following format:

``` markdown
date: 12 Dec 2023
```

I updated the liquid template code to the following. It also formats the posts a bit better on the splash page

```
{% raw %}

{% for post in site.pages %}
{% if post.path contains "docs/" and post.layout == "page" %}
## [{{ post.title }}]({{ post.url }}) <span style="font-size: 0.85em; margin-left: 20px; color: #888;">{{ post.date | date: "%d %b %Y" }}</span>

{{ post.description }}

{% endif %}
{% endfor %}

{% endraw %}
```