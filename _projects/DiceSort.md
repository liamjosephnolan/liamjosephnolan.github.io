---
layout: project
title: 'Dice Sorting Robot'
caption: Using an ABB robot arm and vision processing to sort and place dice
date: '20-04-2024'
image: 
  path: /assets/img/projects/Dicesort.jpg
  srcset: 
    1920w: /assets/img/projects/Dicesort.jpg
    960w:  /assets/img/projects/Dicesort.jpg
    480w:  /assets/img/projects/Dicesort.jpg
sitemap: false
links:
  - title: Link
    url: https://github.com/liamjosephnolan/DiceRobot
---
This project used an ABB robot arm and Cognex camera to track, pickup and sort dice into a tray. The robot's move instructions were all programmed in Rapid with the Vision processing handled by a custom program ran by our Cognex camera. The robot would pick up a dice from a tray, drop it onto the conveyor and then determine where to place to the dice. It would repeat indefinitely or until stopped by the user. It was quite a fun project and was a good exercise in Rapid programming and vision processing. 

Full documentation can be found on my [Dice Sorting] documentation page and Rapid code can be found on my [Github](https://github.com/liamjosephnolan/DiceRobot). 


[Dice Sorting]: /docs/dicesort