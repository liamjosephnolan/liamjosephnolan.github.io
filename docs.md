---
layout: page
title: dev blog
description: >
  i keep documentation for various projects i am working on here
hide_description: true
permalink: /docs/
---

I like to write about what i am working on. here you can find documentation and other info about my current projects.
# Posts

{% assign sorted_posts = site.pages | sort: 'date' | reverse %}
{% for post in sorted_posts %}
  {% if post.path contains "docs/" and post.layout == "page" %}
## [{{ post.title }}]({{ post.url }}) <span style="font-size: 0.85em; margin-left: 20px; color: #888;">{{ post.date | date: "%d %b %Y" }}</span>
{{ post.description }}
  {% endif %}
{% endfor %}



