---
layout: page
title: Basic Webscraper
description: Building a basic web scraper to track the current capacity of my local climbing gym
hide_description: true
date: 27 Oct 2024
---

Alright here is my problem: My climbing gym is too crowded and I need to figure out how I can run my code continuously. 

The "obvious" solution is to build a web scraper to track the climbing gyms capacity (They conveniently display this information of their website)

Its easy to fetch and parse the html content. The current climbing percentage is contained in a snippet like this:

```html
<span class="ki-entry-pct">33</span>
```

So I can just fetch the html content from the url and then use BeautifulSoup to parse and extract the current percentage. Like this:

```python
# Check if the request was successful
if response.status_code == 200:
    # Print the HTML content of the page.
    html_content = response.text
    print("Content Fetched")
else:
    print(f"Failed to retrieve the page. Status code: {response.status_code}")
    html_content = ""

# Create a BeautifulSoup object only if content was fetched
if html_content:
    website_soup = BeautifulSoup(html_content, 'html.parser')

    # Find the span with the class 'ki-entry-pct'
    ki_current_capacity = website_soup.find('span', class_='ki-entry-pct')

    # Extract the text and convert it to an integer if the element is found
    if ki_current_capacity:
        ki_current_capacity = int(ki_current_capacity.get_text())
        print("The extracted number is:", ki_current_capacity)
    else:
        print("The element with class 'ki-entry-pct' was not found.")

```

There are a couple edge case checks in there as well which is nice. Now I need to somehow store this data. I am think either just a text file or csv. 

I will have script just write the current time and capacity and append it to the end of the document. I am going to go with csv as I can always edit it easier in external programs. 

Its pretty easy. I set up the csv file and now I just add the data I have gotten with this:

```python
csv_file = 'ki_current_capacity_log.csv'
current_time = datetime.now() # get current time

# Append data to the CSV file
with open(csv_file, 'a', newline='') as file:
    writer = csv.writer(file)
    writer.writerow([current_time, ki_current_capacity])  
```

Sick! I also timed this script and it runs pretty quick <1second on average. I think its pretty low overhead and apparently I can automate this with github actions? Never used it but lets see!

I created a requirements.txt file with some dependencies I am using

``` text
requests==2.28.1
beautifulsoup4==4.11.1
```

Then I have created a YAML config file

```yaml
name: run webfetch.py

on:
  schedule:
    - cron: '*/5 * * * *'  # This cron expression runs the workflow every 5 minutes.
  workflow_dispatch:  # Allows manual triggering of the workflow

permissions:
  contents: write  # Allow write access to repository contents

jobs:
  run-script:
    runs-on: ubuntu-latest

    steps:
      - name: Checkout Repository
        uses: actions/checkout@v4.2.2

      - name: Set Up Python
        uses: actions/setup-python@v4  # Updated to v4
        with:
          python-version: '3.9'  # Specify the Python version

      - name: Install Dependencies
        run: |
          python -m pip install --upgrade pip
          pip install -r requirements.txt  # Ensure you have a requirements.txt

      - name: Run Script
        run: python webfetch.py  
        
      - name: Configure Git
        run: |
          git config --local user.email "liamjosephnolan@gmail.com"
          git config --local user.name "liamjosephnolan"

      - name: Commit and Push Changes
        run: |
          git add ki_current_capacity_log.csv  
          git commit -m "Update CSV with new capacity data" || echo "No changes to commit"
          git push origin master  # Make sure you push to the correct branch
```


Ok after a bit of debugging the above yaml script should work. I had to add write permission. I am now logging data to my CSV. It seems a little hacky to have these commit processes linked to my GH account. I think there are prewritten actions that GH made for this purpose. 

Im going to leave this running overnight to see if it errors out. I also want to change the scraping processing to getting the boulder and lead capacity. But that is enough for tonight. 




