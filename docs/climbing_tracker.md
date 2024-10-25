---
layout: page
title: Climbing Data Tracker
description: Building a custom training tracking platform to track and report trends/progress in my climbing
hide_description: true
date: 18 Oct 2024
---

Ok here is my vision:

I want to be able to better log my training data while climbing so I can better spot trends in performance (And mostly because this will be a really fun project)

There are quite a few moving parts to this. I'm going to list them here off the top of my head

- **The Data Logger**
	For this application I am going to just create a custom markdown template in Obsidian. I can use it on mobile to record basic stats like: start time, weight, days previously climbed, etc. I can then attach these stats to my actual training metrics: Climbs sent, Hangboard metrics, campus board workouts, etc.
- **The Data Parser**
	I need to write some sort of Python script to scrape through these markdown files and extract and store relevant parameters (Sounds like an excellent time to practice classes and other data structures I never touch)
- **The Backend**
	I need to create some sort of backend service to take all this data and feed it into my website for displaying all this pretty data. This part is definitely a bit of a blackbox to me but it seems exciting to figure out.
- **The Frontend**
	This will be the actual displaying of the data on my website. This might need to be handled on the backend as well. But at the end of the day I want some pretty looking graphs and menus to switch between the data. Should be some fun webdev work

# Data Logger

I've already created a nice sheet in Obsidian that suits my needs. Its a little hacky as Obsidian does not realllllly support dropdown menus so I am using a lot of tables with checkboxes in them. I'm using the "Markdown Table Checkbox" plugin as markdown tables wont naively display checkboxes in tables. This plugin also lets these checkboxes keep their state when toggled in reading mode. They use the HTML element below:

```HTML
<input type="checkbox" unchecked id="angle_30">
```

When they are clicked the *unchecked* state will change to *checked*. You also need make sure each button has a unique id otherwise they will affect the states of others. 

You can see the result below. A bit clunky but I think it is fine.
![[Pasted image 20241018173013.png]]

For hangboard and campus board data entry I will just be using basic tables with text flags to indicate the type of workout. Nothing fancy. 

I also have the following YAML at the top of the file for those important parameters mentioned earlier:

```YAML
---
date: 
start_time: 
end_time: 
weight: 
day_on:
---
```

Ideally I will add some automation using Templater later but for now this will work. Ill add the full files on my Github when I am done

# Data Parser

For this I am just using a Python script. Its still a work in progress but I will document my progress. 

So far I've created a class called **ClimbingSession** where I have those YAML parameters assigned. I use the yaml package and the following to extract these parameters:

```python 
yaml_part = content.split('---')[1].strip()
try:
	config = yaml.safe_load(yaml_part)
	# Access the parameters
	date = config.get('date')
	start_time = config.get('start_time')
	weight = config.get('weight')
	day_on = config.get('day_on')
```

Pretty basic! But getting the kilter parameters is much harder. The markdown file creates this yarbled mess of tables and html. I am currently figuring out how to use the BeautifulSoup package to parse through this. I can extract the snippet I want by looking at the markdown section tags and stripping the content between like this:

```python
# Find the positions of the headers
kilter_start = content.find("# **Kilter**")
hangboard_start = content.find("#  **Hangboard**")

# Extract the content between the two headers
if kilter_start != -1 and hangboard_start != -1:
    html_content = content[kilter_start:hangboard_start].strip()
else:
    print("One of the headers was not found in the content.")

kilter_soup = BeautifulSoup(html_content,'html.parser')
```

That is some pretty good soup but I still have to parse this mess. Luckily it is not too hard. I can use this code snippet

```python
# Find all input elements that have the 'checked' attribute
checked_elements = kilter_soup.find_all(lambda tag: tag.name == "input" and tag.has_attr('checked'))

# Extract and print the ids of the checked elements
checked_ids = [element.get('id') for element in checked_elements]
print(checked_ids)
```

This now outputs the following from my test data

```
['angle_40', 'grade_V7-1', 'grade_V8-1', 'grade_V8-2', 'grade_V8-3', 'grade_V8-4', 'grade_V8-5', 'grade_V8-6', 'grade_V9-1', 'grade_V9-2', 'grade_V9-3', 'grade_V10-1', 'grade_V11-1']
```

Now I just need somehow tally up the angle and grades and assign the total number of each grade to the kilter class. This again is not too hard

First we look at the checked_ids that start with "angle_"

```python
angle_ids = [id for id in checked_ids if id.startswith('angle_')]
```

Now we just check to make sure only one angle was selected and then split that angle to be just a number and store that in kilter.angle

```python
if len(angle_ids) > 1:
    print("Too many angles set")
else:
    kilter.angle = angle_ids[0].split('_')[1]
    
print(f"The angle is: {kilter.angle}")
```