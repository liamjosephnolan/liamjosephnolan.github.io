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

Further Development is still in progress

<h1>Average Capacity Over Time</h1>

<h>Select A Day</h>
<select id="daySelector">
  <option value="" disabled selected>Select a Day</option>
  <option value="Monday">Monday</option>
  <option value="Tuesday">Tuesday</option>
  <option value="Wednesday">Wednesday</option>
  <option value="Thursday">Thursday</option>
  <option value="Friday">Friday</option>
  <option value="Saturday">Saturday</option>
  <option value="Sunday">Sunday</option>
</select>

<h>Or Select A Weather Type</h>

<select id="weatherSelector">
  <option value="" disabled selected>Select Weather Type</option>
  <option value="Sunny">Sunny</option>
  <option value="Cloudy">Cloudy</option>
</select>

{% raw %}
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>

<div>
  <canvas id="myChart"></canvas>
</div>


<script>
  // References to the dropdowns
  const daySelector = document.getElementById('daySelector');
  const weatherSelector = document.getElementById('weatherSelector');

  // Variable to store API data
  let allData = {};
  const today = new Date().toLocaleString('en-US', { weekday: 'long' });

  // Initialize Chart.js chart
  const ctx = document.getElementById('myChart').getContext('2d');
  const chart = new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [{
        label: 'Average Capacity (Percentage)',
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
          max: 100,
          title: { display: true, text: 'Capacity' }
        },
        x: {
          title: { display: true, text: 'Time' },
          min: '09:00:00',
          max: '22:00:00'
        }
      }
    }
  });

  // Function to update the chart with selected data
  function updateChart(key) {
    const keyData = allData[key] || [];
    chart.data.labels = keyData.map(entry => entry.TimeOfDay);
    chart.data.datasets[0].data = keyData.map(entry => entry.Capacity);
    chart.update();
  }

  // Fetch API data and set up dropdown functionality
  fetch('https://ki-webfetch.onrender.com/api/average_capacity')
    .then(response => {
      if (!response.ok) throw new Error('Failed to fetch API data');
      return response.json();
    })
    .then(data => {
      // Store data in the global variable
      allData = data;

      // Event listener for the day selector
      daySelector.addEventListener('change', () => {
        const selectedKey = daySelector.value;

        // Clear the weather selector
        weatherSelector.value = "";

        // Update chart with the selected day's data
        if (allData[selectedKey]) {
          updateChart(selectedKey);
        } else {
          console.warn(`No data available for: ${selectedKey}`);
        }
      });

      // Event listener for the weather selector
      weatherSelector.addEventListener('change', () => {
        const selectedKey = weatherSelector.value;

        // Clear the day selector
        daySelector.value = "";

        // Update chart with the selected weather type's data
        if (allData[selectedKey]) {
          updateChart(selectedKey);
        } else {
          console.warn(`No data available for: ${selectedKey}`);
        }
      });

      // Automatically select today's data if available
      if (allData[today]) {
        daySelector.value = today;
        updateChart(today);
      } else {
        console.warn('No data available for today:', today);
      }
    })
    .catch(error => {
      console.error('Error fetching data:', error);
      document.body.innerHTML += '<h2>Error loading data. Please try again later.</h2>';
    });
</script>
{% endraw %}
