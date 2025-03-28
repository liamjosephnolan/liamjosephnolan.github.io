---
layout: page
title: Weather Frog
description: Updating my weather frog app
hide_description: true
date: 2024-11-08
---
I want to make my Weather Frog app better. Now that I understand API endpoints (And I have stumbled across OpenMeteo) I want to improve the app so it won't require an API key. 

It would be really cool to eventually have it as an installable app and to Open Source license it. The vision remains the same. I want to be able to type in my terminal and have a frog tell me the weather. 

I've started restructing the code already.

I've updated my get_current_weather function to use OpenMeteo. There is just one problem. OM requires coordinates but I want to look up the weather for a city. 

I can use geocode.xyz to convert a city to coords. I wrote a quick function to do this:


```python
def convert_city_coords(city): # Function to convert city name to coordinates
    url = f'https://geocode.xyz/{city}?json=1'
    response = requests.get(url)
    data = response.json()
    latitude = data['latt']
    longitude = data['longt']
    return latitude, longitude
```

