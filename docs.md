---
layout: page
title: Dev Blog
description: >
  Here I will added documentation for various projects I am working on
hide_description: true
permalink: /docs/
---

Here you can find project documentation, source code, and other fun content.

## Dev Blog Posts


{% for post in site.pages %}
{% if post.path contains "docs/" and post.layout == "page" %}
* [{{ post.title }}]({{ post.url }}) - {{ post.description }}
{% endif %}
{% endfor %}

[Thesis]: /docs/thesis.md
[website]: /docs/website.md
[email]: /docs/email.md
[Edge Detection]: /docs/turtlesim.md
[Dice Sorting]: /docs/dicesort.md



