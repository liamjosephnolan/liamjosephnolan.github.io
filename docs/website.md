---
layout: page
title: Website
description: >
  Website doc page
hide_description: true
---

How did I create such a beautiful site with such responsive elements? I didn't! I used Jekyll and its associated themes to do a 
lot of the heavy lifting. This site is powered by Hydejack, a really nice looking portfolio intended theme. Once I figured out how to write
markdown files it was pretty easy. There are some custom HTML elements I wrote that I will talk about here. 

## Form Spring Contact Form

I am using formspring for the contact form, its free and pretty easy to setup. The HTML source code is below:

~~~js
<form target="_blank" method="post" action="https://formsubmit.co/liamjosephnolan@gmail.com">
  <div class="form-group">
    <label for="exampleFormControlInput1">Your Name</label>
    <input type="text" class="form-control" id="exampleFormControlInput1" placeholder="First Last">
  </div>
  <div class="form-group">
    <label for="exampleFormControlInput1">Your Email address</label>
    <input type="email" class="form-control" id="exampleFormControlInput1" placeholder="name@example.com">
  </div>
  <div class="form-group">
    <label for="exampleFormControlTextarea1">Your Message</label>
    <textarea class="form-control" id="exampleFormControlTextarea1" rows="3"></textarea>
  </div>
    <div>
        <ul class="actions">
            <button type="submit" class="btn btn-primary">Submit</button>
        </ul>
    </div>
</form>
~~~

Formspring Source Code
{:.figcaption}