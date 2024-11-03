---
layout: default
title: Average Capacity Chart
---

<canvas id="myChart"></canvas>

{% raw %}
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<script>
    const API_URL = 'https://ki-webfetch.onrender.com/api/average_capacity';
    fetch(API_URL)
        .then(response => {
            if (!response.ok) {
                throw new Error('Network response was not ok');
            }
            return response.json();
        })
        .then(data => {
            const labels = data.map(entry => entry.TimeOfDay);
            const capacities = data.map(entry => entry.Capacity);

            const ctx = document.getElementById('myChart').getContext('2d');
            const myChart = new Chart(ctx, {
                type: 'line',
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
{% endraw %}

