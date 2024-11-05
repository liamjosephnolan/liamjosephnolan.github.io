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
accent_color: '#4fb1ba'
accent_image:
  background: '#193747'
theme_color: '#193747'
sitemap: false
---

My local climbing gym displays it's current capacity in real time on their website. I built a webscraper in Python that parses the webcode and stores the current capacity into a CSV file. I then automated this script using Github acitons to run every 10 minutes. 

I also created a Flask app in Python that parses this CSV file, averages the data for each day of the week and then returns a JSON export. I doockerized this codebase and deployed it on Render as an API endpoint.

I then wrote an HTML script to plot and display all this data on my website which you can see below.

This project taught me a ton about CI/CD deployment and Docker while also providing me data about the best time to go climbing. The full codebase can be found on my [Github](https://github.com/liamjosephnolan/ki_webfetch) and for additional info you can read my [Dev Blog Post](https://liamjosephnolan.com/docs/web_scraper/). 


<h1>Average Capacity Over Time</h1>

<!-- Day Selector Dropdown -->
<select id="daySelector">
    <option value="">Select a Day</option>
</select>

<canvas id="myChart"></canvas>

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
    select {
        margin-bottom: 20px;
        font-size: 16px;
        padding: 5px;
    }
</style>

{% raw %}
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>

<script>
    const API_URL = 'https://ki-webfetch.onrender.com/api/average_capacity';

    let chart;
    let allData = {};

    const daysInOrder = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"];

    fetch(API_URL)
        .then(response => {
            if (!response.ok) throw new Error('Network response was not ok');
            return response.json();
        })
        .then(data => {
            allData = data.reduce((acc, day) => {
                acc[day.DayOfWeek] = day.Data;
                return acc;
            }, {});

            const daySelector = document.getElementById('daySelector');
            daysInOrder.forEach(day => {
                if (allData[day]) {
                    const option = document.createElement('option');
                    option.value = day;
                    option.textContent = day;
                    daySelector.appendChild(option);
                }
            });

            const ctx = document.getElementById('myChart').getContext('2d');
            chart = new Chart(ctx, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [{
                        label: 'Average Capacity',
                        data: [],
                        borderColor: 'rgba(75, 192, 192, 1)',
                        backgroundColor: 'rgba(75, 192, 192, 0.2)',
                        borderWidth: 2,
                        fill: true
                    }]
                },
                options: {
                    responsive: true,
                    scales: {
                        y: {
                            beginAtZero: true,
                            title: { display: true, text: 'Capacity' }
                        },
                        x: {
                            title: { display: true, text: 'Time' }
                        }
                    }
                }
            });

            daySelector.addEventListener('change', event => {
                const selectedDay = event.target.value;
                if (selectedDay) updateChart(selectedDay);
            });
        })
        .catch(error => {
            console.error('Error fetching data:', error);
            document.body.innerHTML = '<h2>Error loading data</h2>';
        });

    function updateChart(day) {
        const dayData = allData[day] || [];
        const labels = dayData.map(entry => entry.TimeOfDay);
        const capacities = dayData.map(entry => entry.Capacity);

        chart.data.labels = labels;
        chart.data.datasets[0].data = capacities;
        chart.update();
    }
</script>
{% endraw %}


