---
layout: project
title: 'Delta Manipulator Simulation'
caption: Simulating Delta Manipulator using Matlab and Simscape Multibody
date: '20-04-2024'
image: 
  path: /assets/img/projects/Delta.png
  srcset: 
    1920w: /assets/img/projects/Delta.png
    960w:  /assets/img/projects/Delta.png
    480w:  /assets/img/projects/Delta.png
sitemap: false
# links:
#   - title: Link
#     url: https://github.com/liamjosephnolan/Delta-Manipulator
---
This project used Matlab, Simulink, and Simscape Multibody to model and simulate an IGUS Delta Manipulator. A desired trajectory path for the end effector was planned using trapezoidal velocity profiles. Inverse kinematics were then used to calculate joint positions based on the desired trajectory. Forward kinematics were then used to calculate the new end effector position and update the Simscape model. 

<!-- All relevant files can be found on my [Github](https://github.com/liamjosephnolan/Delta-Manipulator) and the full project report can be found below. -->


<!-- 
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
    <embed id="pdf-embed" src="/assets/img/projects/Liam_Nolan_Delta_Manipulator.pdf" type="application/pdf" width="700px" height="700px"/>
</body>
</html> -->


