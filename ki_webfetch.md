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

