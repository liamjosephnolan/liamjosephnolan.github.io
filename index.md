---
layout: welcome
title: Welcome
cover: true
selected_projects:
- _projects/DiceSort.md
- _projects/EdgeDetect.md
projects_page: projects.md
---

<div style="display: flex; align-items: stretch; margin: 20px 0; padding: 20px; border: 1px solid #444; border-radius: 0; background-color: #3c3836; color: #fff;">
  <!-- About Text -->
  <div style="flex: 1; padding: 20px;">
    <h2 style="margin-top: 0; color: #fff;">About Me</h2>
    <p>
      I am a Robotics Engineer with four years of professional experience in surgical robotics. 
      I am currently writing my Master's thesis on using adaptive control methods in ROS 2 for affordable surgical robotics.
    </p>
    <p>
      I am passionate about solving challenging problems in robotics, automation, and control systems. 
      In my free time, I enjoy working on side projects, climbing, and playing with robots.
    </p>
  </div>

  <!-- Avatar Image -->
  <div class="avatar-container" style="flex: 0 0 300px; background-image: url('/assets/img/avatar.JPEG'); background-size: cover; background-position: center;">
  </div>
</div>

<style>
  /* Hide the avatar image on screens smaller than 768px */
  @media screen and (max-width: 768px) {
    .avatar-container {
      display: none;
    }
  }
</style>

## Side Projects
Curious about what I do in my spare time? Here are some side projects I have worked on.

<!--projects-->

## Resume
Want to know a bit more about my professional experience and skills? This site also hosts my [Resume]{:.heading.flip-title}

<!--html element to embed pdf resume and hide on mobile (Because it looks bad)-->

<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>PDF Embed</title>
    <style>
        /* Hide the PDF on screens smaller than 768px (standard tablet size) */
        @media screen and (max-width: 768px) {
            #pdf-embed {
                display: none;
            }
        }
    </style>
</head>
<body>
    <embed id="pdf-embed" src="/assets/LiamNolanCV.pdf" type="application/pdf" width="700px" height="700px"/>
</body>
</html>

[Resume]: /resume/
