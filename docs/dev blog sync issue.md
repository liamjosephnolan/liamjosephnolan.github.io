---
layout: page
title: Dev blog sync issue
description: Automatically updating my dev blog splash page
hide_description: true
---
# Dev Blog Sync Issue 

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
	
	

Ok so apparently I can use something called liquid templating. It would look something like this 

```
## Blog Posts {% for post in site.posts %} * [{{ post.title }}]({{ post.url }}) - {{ post.date | date: "%B %d, %Y" }} {% endfor %}

```

I can grab the **page** tag capture all my dev blog posts like this

```md
## Blog Posts {% for post in site.pages %} {% if post.layout == "page" and post.title != "docs" %} * [{{ post.title }}]({{ post.url }}) - {{ post.description }} {% endif %} {% endfor %}
```

Testing it now but I also forget how to build the page. For future reference here is the command and its hosted at http://localhost:4000

```bash
bundle exec jekyll serve
```

Ok that worked a bit too well. Turns out there are a bunch of irrevelant pages with the page tag 


I update the liquid part to this and it worked Perfect!! 

```
{% for post in site.pages %}
{% if post.path contains "docs/" and post.layout == "page" %}
* [{{ post.title }}]({{ post.url }}) - {{ post.description }}
{% endif %}
{% endfor %}
```

The main thing is now in my blog posts I need to make sure the description is good because now it user facing. Easy enough fix for the future. 

These notes will now become my first test post! Ill even put a test image below to see if that breaks it. 

![[fern.jpg]]
