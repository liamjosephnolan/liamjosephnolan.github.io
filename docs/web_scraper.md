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

Ok I left it running for a week now. and I have done a lot in that time. I was right that it would error out overnight. What happens is when KI is closed, the html element is just an emoji which breaks my code. I fixed the web scraper to handle this edge case. The ey was just adding a try block and setting the capacity to 0 if there is an exception.

```python
    # Extract the text and convert it to an integer if the element is found
    if ki_current_capacity:
        try:
            ki_current_capacity = int(ki_current_capacity.get_text())
            print("The extracted number is:", ki_current_capacity)
        except:
            ki_current_capacity = 0
    else:
        print("The element with class 'ki-entry-pct' was not found.")
        ki_current_capacity = 0

```

Also KI actually keeps the same hours every single day (9:00-22:00). This is handy because I can schedule my Cron job to only run during these hours saving computational time for my github actions. I've updated the actions.yml file to only running during these times with this block

```yaml
on:
  schedule:
    - cron: '*/10 8-21 * * *'  # This cron expression runs the workflow every 10 minutes.
```

Note that this is is for CET during daylight savings time. This will be offset by an hour when daylight savings ends but that is in 6 months because it just started. Will I fix it by then? Probably not? But I should have way too much data by then anyways. 

For some reason my Cron job isn't running at precise 10 minute intervals starting at KI's opening and it also runs an hour past closing. I think Github Actions is not intended to be running this frequently with this sort of percision. This will need to be fixed later.

For now I have 382 data points and I want to do something with this data. I decided that I wanted to have this data displayed in a nice little web app on my website which required a bunch of fun new code. After researching a bit I landed on the following workflow:

1. Github Actions will run my webscraper and update the CSV file
2. A python flask app running on Render will parse this CSV file and average capacity over each 15 minute interval. It will then JSONify this and push it to an API end point on render
3. Some HTML code will plot and display this data on my website

Is it over engineered? Yes! Will it let me practice Docker and back-end development! Also yes!

I've gone ahead and already done all this. First I wrote a python flask app to fetch the CSV file and parse it. This was my first time doing this but it was pretty simple.

First I wrote a function that averages the capacity data and returns it as Json data 

```python
def average_capacity():

    csv_file = 'ki_current_capacity_log.csv'
    
    def calculate_average_capacity_by_interval(csv_file):
        df = pd.read_csv(csv_file)
        df['Timestamp'] = pd.to_datetime(df['Timestamp'])
        df['TimeOfDay'] = df['Timestamp'].dt.floor('15T').dt.time
        start_time = pd.to_datetime("08:00").time()
        end_time = pd.to_datetime("21:00").time()
        df = df[(df['TimeOfDay'] >= start_time) & (df['TimeOfDay'] <= end_time)]
        average_capacity = df.groupby('TimeOfDay')['Capacity'].mean().reset_index()
        average_capacity['TimeOfDay'] = average_capacity['TimeOfDay'].astype(str)
        return average_capacity.to_dict(orient='records')
    
    averages = calculate_average_capacity_by_interval(csv_file)
    return jsonify(averages)
```

I hard coded the start and end times which isn't great but it works for my application. 

Next I have to make this into a flask app, which is also pretty simple. One bug I ran into is I had to enable CORS otherwise you can't access the JSON data (Some security measure I guess)

```python

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes


@app.route('/api/average_capacity', methods=['GET'])


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)
```

The whole app is only 30 lines of code

```python
from flask import Flask, jsonify
import pandas as pd
from flask_cors import CORS  # Import CORSapp = Flask(__name__)


app = Flask(__name__)
CORS(app)  # Enable CORS for all routes


@app.route('/api/average_capacity', methods=['GET'])
def average_capacity():
    # Path to your CSV file
    csv_file = 'ki_current_capacity_log.csv'
    
    def calculate_average_capacity_by_interval(csv_file):
        df = pd.read_csv(csv_file)
        df['Timestamp'] = pd.to_datetime(df['Timestamp'])
        df['TimeOfDay'] = df['Timestamp'].dt.floor('15T').dt.time
        start_time = pd.to_datetime("08:00").time()
        end_time = pd.to_datetime("21:00").time()
        df = df[(df['TimeOfDay'] >= start_time) & (df['TimeOfDay'] <= end_time)]
        average_capacity = df.groupby('TimeOfDay')['Capacity'].mean().reset_index()
        average_capacity['TimeOfDay'] = average_capacity['TimeOfDay'].astype(str)
        return average_capacity.to_dict(orient='records')
    
    averages = calculate_average_capacity_by_interval(csv_file)
    return jsonify(averages)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)
```

In order to run this app on Render I will need to Dockerize it which requires me to create a Dockerfile. Its pretty basic, I just specify my python version, set the working directory, point towards the requirements.txt file (Which I updated with relevant libraries), run pip, copy the rest of the code, expose the access port, and then actually run the file.

```Docker
# Use the official Python image from Docker Hub
FROM python:3.10-slim

# Set the working directory
WORKDIR /app

# Copy requirements.txt and install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy the rest of the application code
COPY . .

# Expose the port Flask will run on
EXPOSE 8080

# Run the Flask application
CMD ["python", "app.py"]
```

I successfully ran it locally and could see all the JSON data but I wanted it running on Render. I just uploaded all of this to Github and then pointed Render towards the relevant directory. Now if I go to https://ki-webfetch.onrender.com/api/average_capacity I can see my JSON data. I am probably doing this in a horribly insecure manor but It doesn't really matter.

Now I can just add this HTML snippet to a page on my website and it will plot a nice little chart of all the data

```HTML
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Average Capacity Chart</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 20px;
            display: flex;
            flex-direction: column;
            align-items: center;
        }
        canvas {
            max-width: 800px;
            width: 100%;
            height: auto;
        }
    </style>
</head>
<body>

<h1>Average Capacity Over Time</h1>
<canvas id="myChart"></canvas>

<script>
    // Replace this with your actual API endpoint
    const API_URL = 'https://ki-webfetch.onrender.com/api/average_capacity';

    // Fetch the JSON data from the API
    fetch(API_URL)
        .then(response => {
            if (!response.ok) {
                throw new Error('Network response was not ok');
            }
            return response.json();
        })
        .then(data => {
            // Prepare the labels and data for the chart
            const labels = data.map(entry => entry.TimeOfDay);
            const capacities = data.map(entry => entry.Capacity);

            // Create the chart
            const ctx = document.getElementById('myChart').getContext('2d');
            const myChart = new Chart(ctx, {
                type: 'line', // Change to 'bar' for a bar chart, etc.
                data: {
                    labels: labels,
                    datasets: [{
                        label: 'Average Capacity',
                        data: capacities,
                        borderColor: 'rgba(75, 192, 192, 1)',
                        backgroundColor: 'rgba(75, 192, 192, 0.2)',
                        borderWidth: 2,
                        fill: true,
                    }]
                },
                options: {
                    responsive: true,
                    scales: {
                        y: {
                            beginAtZero: true,
                            title: {
                                display: true,
                                text: 'Capacity'
                            }
                        },
                        x: {
                            title: {
                                display: true,
                                text: 'Time'
                            }
                        }
                    }
                }
            });
        })
        .catch(error => {
            console.error('Error fetching data:', error);
            document.body.innerHTML = '<h2>Error loading data</h2>';
        });
</script>

</body>
</html>
```

Here is where I run into some issues. I had this up and running but now on my website it just shows a blank page. If I host the website locally I can see my beautiful chart. Poking around in developer tools shows a bunch of green 200 status codes so there isn't an obvious issue. 

My API end point is fine, I can still see it. I think there might be a security in my CORS. I am going to specify my origin access to only be my website like this

```python
CORS(app, resources={r"/api/*": {"origins": "https://liamjosephnolan.com"}})
```

It still works locally but I have to wait for my website to update.....

Well that did not work >:( but at least my API is slightly more secure. I think it might be an issue with Jekyll/Markdown. I created a normal html page without and styling and put the html snippet it. It wont work locally because I changed the CORS allow origin snippet. I think I will revert that.

It works locally now and I just get an unstyled html page with the chart

