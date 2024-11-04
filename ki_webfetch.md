<h1>Average Capacity Over Time</h1>

<!-- Day Selector Dropdown -->
<select id="daySelector">
    <option value="">Select a Day</option>
</select>

<canvas id="myChart"></canvas>

<!-- Load Chart.js -->
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>

<script>
    // Your Flask API endpoint
    const API_URL = 'https://ki-webfetch.onrender.com/api/average_capacity';

    let chart;
    let allData = {}; // Store all data once fetched

    // Array with days in chronological order
    const daysInOrder = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"];

    // Fetch the JSON data from the API
    fetch(API_URL)
        .then(response => {
            if (!response.ok) {
                throw new Error('Network response was not ok');
            }
            return response.json();
        })
        .then(data => {
            // Store the fetched data, organizing by day
            allData = data.reduce((acc, day) => {
                acc[day.DayOfWeek] = day.Data;
                return acc;
            }, {});

            // Populate the day selector dropdown in chronological order
            const daySelector = document.getElementById('daySelector');
            daysInOrder.forEach(day => {
                if (allData[day]) { // Only add days that exist in data
                    const option = document.createElement('option');
                    option.value = day;
                    option.textContent = day;
                    daySelector.appendChild(option);
                }
            });

            // Initialize an empty chart
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

            // Add event listener for day selection
            daySelector.addEventListener('change', event => {
                const selectedDay = event.target.value;
                if (selectedDay) {
                    updateChart(selectedDay);
                }
            });
        })
        .catch(error => {
            console.error('Error fetching data:', error);
            document.body.innerHTML = '<h2>Error loading data</h2>';
        });

    // Function to update the chart based on the selected day
    function updateChart(day) {
        const dayData = allData[day] || [];
        const labels = dayData.map(entry => entry.TimeOfDay);
        const capacities = dayData.map(entry => entry.Capacity);

        chart.data.labels = labels;
        chart.data.datasets[0].data = capacities;
        chart.update();
    }
</script>

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
