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

