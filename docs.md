---
layout: page
title: Dev Blog
description: >
  Here I will added documentation for various projects I am working on
hide_description: true
permalink: /docs/
---

Here you can find project documentation, source code, and other fun content.

# Posts
{% for post in site.pages %}
{% if post.path contains "docs/" and post.layout == "page" %}
## [{{ post.title }}]({{ post.url }}) <span style="font-size: 0.85em; margin-left: 20px; color: #888;">{{ post.date | date: "%d %b %Y" }}</span>

{{ post.description }}

{% endif %}
{% endfor %}

