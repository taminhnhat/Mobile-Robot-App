const chartSize = 20;
var chartData_front_right_wheel = new Array(chartSize).fill(0);
var chartData_rear_right_wheel = new Array(chartSize).fill(0);
var chartData_rear_left_wheel = new Array(chartSize).fill(0);
var chartData_front_left_wheel = new Array(chartSize).fill(0);
var linear_vel_chart = new Array(chartSize).fill(0)
var angular_vel_chart = new Array(chartSize).fill(0)

var linearVelocityChart = new Chart("linearVelocityChart", {
    type: "line",
    data: {
        labels: new Array(chartSize).fill(0),
        datasets: [{
            fill: false,
            lineTension: 0,
            backgroundColor: "rgba(0,0,255,1.0)",
            borderColor: "rgba(0,0,255,0.1)",
            data: new Array(chartSize).fill(0)
        }]
    },
    options: {
        legend: { display: false },
        scales: {
            yAxes: [{ ticks: { min: -0.5, max: 0.5 } }],
        },
        overrides: {
            scales: true
        }
    }
});

var angularVelocityChart = new Chart("angularVelocityChart", {
    type: "line",
    data: {
        labels: new Array(chartSize).fill(0),
        datasets: [{
            fill: false,
            lineTension: 0,
            backgroundColor: "rgba(0,0,255,1.0)",
            borderColor: "rgba(0,0,255,0.1)",
            data: new Array(chartSize).fill(0)
        }]
    },
    options: {
        legend: { display: false },
        scales: {
            yAxes: [{ ticks: { min: -2.5, max: 2.5 } }],
        }
    }
});

var linearMiniChart = new Chart("linearMiniChart", {
    type: 'doughnut',
    data: {
        datasets: [{
            backgroundColor: [
                "#23D160",
                "#FF3860"
            ],
            data: [0, 100],
            borderWidth: [0, 0],
        }]
    },
    options: {
        legend: {
            display: false,
        },
        cutoutPercentage: 40,
        tooltip: {
            enabled: false,
        }
    }
});

var angularMiniChart = new Chart("angularMiniChart", {
    type: 'doughnut',
    data: {
        datasets: [{
            backgroundColor: [
                "#23D160",
                "#FF3860"
            ],
            data: [0, 100],
            borderWidth: [0, 0],
        }]
    },
    options: {
        legend: {
            display: false,
        },
        cutoutPercentage: 40,
        tooltip: {
            enabled: false,
        }
    }
});

var wheelVelocityChart = new Chart("wheelVelocityChart", {
    type: "line",
    data: {
        labels: new Array(chartSize).fill(0),
        datasets: [{
            fill: false,
            lineTension: 0,
            backgroundColor: "rgba(254, 119, 123, 0.5)",
            borderColor: "rgba(254, 119, 123, 0.5)",
            data: new Array(chartSize).fill(0)
        },
        {
            fill: false,
            lineTension: 0,
            backgroundColor: "rgba(255, 206, 86, 0.5)",
            borderColor: "rgba(255, 206, 86, 0.5)",
            data: new Array(chartSize).fill(1)
        },
        {
            fill: false,
            lineTension: 0,
            backgroundColor: "rgba(54, 162, 235, 0.5)",
            borderColor: "rgba(54, 162, 235, 0.5)",
            data: new Array(chartSize).fill(2)
        },
        {
            fill: false,
            lineTension: 0,
            backgroundColor: "rgba(75, 192, 192, 0.5)",
            borderColor: "rgba(75, 192, 192, 0.5)",
            data: new Array(chartSize).fill(3)
        }]
    },
    options: {
        legend: { display: false },
        scales: {
            yAxes: [{ ticks: { min: -8.0, max: 8.0 } }],
        },
        overrides: {
            scales: true
        },
        elements: {
            point: {
                radius: 0
            }
        }
    }
});