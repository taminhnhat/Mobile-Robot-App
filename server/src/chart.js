const xValues = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11];
const yValues = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

var linearVelocityChart = new Chart("myChart", {
    type: "line",
    data: {
        labels: xValues,
        datasets: [{
            fill: false,
            lineTension: 0,
            backgroundColor: "rgba(0,0,255,1.0)",
            borderColor: "rgba(0,0,255,0.1)",
            data: linear_vel_chart
        }]
    },
    options: {
        legend: { display: false },
        scales: {
            yAxes: [{ ticks: { min: -0.5, max: 0.5 } }],
        }
    }
});

// setInterval(() => {
//     yValues.shift()
//     yValues.push(Math.round(Math.random() * 16))
//     velocityChart.data.datasets[0].data = linear_vel_chart
//     velocityChart.update()
// }, 1000)