---
layout: page
title: Dev Blog

permalink: /docs/
---

I often blog/document my persoanl projects while I work on them. This serves as a repository for these posts.

# Posts

<ul>
  {% for page in site.pages %}
    {% if page.path contains 'blog/' %}
      <li><a href="{{ page.url }}">{{ page.title | default: page.url }}</a></li>
    {% endif %}
  {% endfor %}
</ul>