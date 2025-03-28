---
layout: page
title: Dev Blog

permalink: /docs/
---

I like to write about what I'm working on. Here's documentation and other info about my current projects.

# Posts

<ul>
  {% for page in site.pages %}
    {% if page.path contains 'docs/' %}
      <li><a href="{{ page.url }}">{{ page.title | default: page.url }}</a></li>
    {% endif %}
  {% endfor %}
</ul>