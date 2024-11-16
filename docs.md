---
layout: page
title: Dev Blog
description: >
  I keep documentation for various projects I am working on here
hide_description: true
permalink: /docs/
---

I like to write about what I am working on. Here you can find documentation and thoughts about what I am currently working on

# Posts
{% for post in site.pages %}
{% if post.path contains "docs/" and post.layout == "page" %}
## [{{ post.title }}]({{ post.url }}) <span style="font-size: 0.85em; margin-left: 20px; color: #888;">{{ post.date | date: "%d %b %Y" }}</span>

{{ post.description }}

{% endif %}
{% endfor %}

