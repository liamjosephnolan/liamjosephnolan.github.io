---
layout: project
title: 'Futura Pendulum'
caption: Controls Project to optimize the control and balancing of a futura style pendulum 
date: 01 March 2025 
image: 
  path: /assets/img/projects/ki_webfetch.png
  srcset: 
    1920w: /assets/img/projects/system.png
    960w:  /assets/img/projects/system.png
    480w:  /assets/img/projects/system.png
links:
  - title: Github
    url: https://github.com/liamjosephnolan/multbody_proj/tree/main/Task_III_solution
accent_color: '#4fb1ba'
accent_image:
  background: '#193747'
theme_color: '#193747'
sitemap: false
---

A Futura pendulum is an adaptation of the classic pendulum-on-a-cart problem. However, rather than having the revolute joint of the pendulum connected to a linear cart, the revolute joint is connected to a second revolute joint. This unique formulation allows for theoretically infinite angular rotation of the base joint, enabling unique motion possibilities.

The pendulum was modeled using Simscape Multibody, and a Linear Quadratic Regulator (LQR) controller was designed in Simulink to stabilize the pendulum in its upright position. MATLAB's linearization toolbox was used to linearize the system around its upright position, and the gain matrix was calculated from this linearization.

![Simulink Model](/assets/img/projects/model.png "Simulink Model")

Overall, the system proved to be quite stable, responding well to both initial displacement and further perturbations. This project proved to be an excellent learning experience in modeling and controlling unstable systems using Simscape Multibody.

![System dynamics response](/assets/img/projects/response.png "System dynamics response")