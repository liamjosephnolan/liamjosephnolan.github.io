---
layout: project
title: 'Python Webscraper'
caption: Software project to collect, track, and display climbing gym capacity 
date: 1 Nov 2024 
image: 
  path: /assets/img/projects/ki_webfetch.png
  srcset: 
    1920w: /assets/img/projects/ki_webfetch.png
    960w:  /assets/img/projects/ki_webfetch.png
    480w:  /assets/img/projects/ki_webfetch.png
links:
  - title: Link
    url: https://github.com/liamjosephnolan/ki_webfetch
sitemap: false
---

My local climbing gym displays it's current capacity in real time on their website. I built a webscraper in Python that parses the webcode and stores the current capacity into a CSV file. I then automated this script using Github acitons to run every 10 minutes. 

I also created a Flask app in Python that parses this CSV file, averages the data for each day of the week and then returns a JSON export. I doockerized this codebase and deployed it on Render as an API endpoint.

I then wrote an HTML script to plot and display all this data on my website which you can see below.

This project taught me a ton about CI/CD deployment and Docker while also providing me data about the best time to go climbing. The full codebase can be found on my [Github](https://github.com/liamjosephnolan/ki_webfetch) and for additional info you can read my [Dev Blog Post](https://liamjosephnolan.com/docs/web_scraper/).

I host the data on my website [here](https://liamjosephnolan.com/epic) in all it's 90's style HTML glory.