// data and sensors_dict variables sent from views.py
data = JSON.parse(data);
// sensors_dict is data from QT
dataSensor = JSON.parse(sensors_dict);
console.log(dataSensor);
console.log(data);  



var ultraCharts = []
// Charts drawing loop function
function display(sensor_name, number_of_sensors) {

    for (let i = 1; i <= number_of_sensors; i++) {
        $("#sensors").append(
            '<div class="col-lg-4">' +
            '<div class="card card-chart"> ' +
            '<div class="card-header">' +
            ' <h3 class="card-title"><i style="color:#E14FCB" class="tim-icons icon-bus-front-12"></i>  ' + sensor_name + i + '</h3>' +
            '</div>' +
            '<div class="card-body">' +
            ' <canvas id="' + sensor_name + i + 'Chart" style=" height: 345px; width: 476px;"></canvas>' +
            '</div> ' +
            ' </div> ' +
            '</div>');
    }

    for (let i = 1; i <= number_of_sensors; i++) {
        $("#sensorsCol").append(
            '<th>' + sensor_name + i + '</th>'
            );
    }

    var options = {
        maintainAspectRatio: false,
        legend: {
            display: true
        },
        responsive: true,
        title: {
            display: true,
            // text: 'Sensing Data'
        },
        tooltips: {
            mode: 'index',
            intersect: false,
        },
        hover: {
            mode: 'nearest',
            intersect: true
        },
        scales: {
            xAxes: [{
                display: true,
                scaleLabel: {
                    display: true,
                    labelString: 'Time'
                },
                ticks: {
                    display: false //this will remove only the label
                }
            }],
            yAxes: [{
                display: true,
                scaleLabel: {
                    display: false,
                    labelString: "" + sensor_name
                }
            }]
        }
    }
    for (let i = 1; i <= number_of_sensors; i++) {

        var ctx = $('#' + sensor_name + i + 'Chart')[0].getContext('2d');
        ultraCharts.push(new Chart(ctx, {
            // The type of chart we want to create
            type: 'line',

            // The data for our dataset
            data: {
                labels: [],
                datasets: [
                    {
                        label: "" + sensor_name + i,
                        backgroundColor: '#E14FCB',
                        borderColor: '#E14FCB',
                        data: [],
                        pointHoverRadius: 4,
                        pointRadius: 2,
                        borderWidth: 2,
                        fill: false
                    }
                ],
            },

            // Configuration options go here
            options: options
        }))
    }
}

for (const [key, value] of Object.entries(dataSensor)) {
    display(key, value)
}

$('#form').on('submit', function (e) {

    e.preventDefault();

    $.ajax({
        type: "POST",
        url: "{% url 'show_data' %}",
        data: {
            FromDate: $('#FromDate').val(),
            ToDate: $('#ToDate').val(),
            csrfmiddlewaretoken: window.CSRF_TOKEN,
            dataType: "json",

        },

        success: function (data) {
            console.log("Success")
            data = JSON.parse(data)

         
            // dummy_list should be refactored to data variable sent from DB
            timeDataList = data["time"];
            delete data["time"];

            // Set Time Labels
            for(let i = 0 ; i < (ultraCharts.length); i++){
                ultraCharts[i].data.labels = timeDataList;
            }

            // Set values to be plotted reading from the dictionary sent from database
            for(let i = 0 ; i < (ultraCharts.length); i++){
                // the key is the same name as the chart name
                // dummy_list should be refactored to data variable sent from DB
                ultraCharts[i].data.datasets[0].data  = data[ultraCharts[i].data.datasets[0].label];
                console.log(ultraCharts[i].data.datasets[0].data);
            }
          
            // Update charts with new values
            for(let i = 0 ; i < (ultraCharts.length); i++){
                ultraCharts[i].update();
            }
            
        },
        failure: function () {
            console.log("fail")

        }
    });


});