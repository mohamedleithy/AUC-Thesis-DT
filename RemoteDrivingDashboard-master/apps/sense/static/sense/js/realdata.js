
dataSensor = JSON.parse(dataSensorInfo);

anomaly = JSON.parse(anomalyData);

// console.log(anomaly);

const socket = io('http://localhost:8000/web');
    console.log(socket)
    socket.on('connect', () => {
        console.log(`connect ${socket.id}`);
    });

    socket.on('disconnect', (data) => {
        console.log(`disconnect ${socket.id}`);
    });

var ultraCharts = []

function display (sensor_name, number_of_sensors){

    for (let i =1;i<=number_of_sensors;i++){
        $("#sensors").append(
            '<div class="col-lg-4">'+
               '<div class="card card-chart"> ' +
                    '<div class="card-header">' +
                       ' <h3 class="card-title"><i style="color:#E14FCB" class="tim-icons icon-bus-front-12"></i>  ' + sensor_name +'_'+ i +'</h3>'  +
                    '</div>' +
                    '<div class="card-body">' +
                       ' <canvas id="'+sensor_name+i+'Chart" style=" height: 345px; width: 476px;"></canvas>' +
                    '</div> '+
               ' </div> '+
            '</div>');
        }

    var options =  {
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
                    labelString: ""+sensor_name
                }
            }]
        }
    }    
    for (let i =1;i<=number_of_sensors;i++){

        var ctx = $('#'+sensor_name+i+'Chart')[0].getContext('2d');
        ultraCharts.push( new Chart(ctx, {
                // The type of chart we want to create
                type: 'line',
                // The data for our dataset
                data: {
                    labels: [],
                    datasets: [
                    {
                        label: ""+ sensor_name+i,
                        backgroundColor: '#E14FCB',
                        borderColor: '#E14FCB',
                        data: [],
                        pointHoverRadius: 4,
                        pointRadius: 2,
                        borderWidth: 2,
                        fill: false,

                    }
                    ],
                },
        
                // Configuration options go here
                options:options
            }))
        }
}
var totalNumOfSensors = 0
for (const [key, value] of Object.entries(dataSensor)){
    console.log(key);
    display(key, value)
    //calculates the total num, example -> (3 ultrasonic + 1 speed + 1 temp) = 6
    totalNumOfSensors += value
}

    socket.on('sensedData', (data) =>{
        //timestamp here ---------------------------------
        var date = new Date();
        var timestamp = date.getTime();

        var difference = timestamp - data["time"];

        console.log('Difference: ', difference);
  
        console.log(data);


        

        var name = data["sensor_name"];

        for (let i=0;i<totalNumOfSensors;i++){
            if(ultraCharts[i].data.datasets[0].label==name){
                ultraCharts[i].data.labels.push(data["time"]);
            }
        }

        //save the timestamp into csv file //boody

        
        delete data["sensor_name"]
        delete data["time"]
        // delete data ['lng']
        // delete data ['lat']

        // let i = 0
        // for (const [key, value] of Object.entries(data)){
        //     console.log(data[key]);
        //     console.log("key", key);

        //     console.log(ultraCharts[i].data.datasets[0].label, " ", name);
        //     if(ultraCharts[i].data.datasets[0].label==name){
        //         // console.log("draw");
        //         ultraCharts[i].data.datasets[0].data.push(data[key]);
        //         ultraCharts[i].update();
        //     }
        //     i++;
        // }

        for (let i=0;i<totalNumOfSensors;i++){

            console.log(ultraCharts[i].data.datasets[0].label, " ", name);
            if(ultraCharts[i].data.datasets[0].label==name){
                // console.log("draw");
                ultraCharts[i].data.datasets[0].data.push(data["magnitude"]);
                ultraCharts[i].update();
            }
        }




});


 $(document).ready(function(){
  $('#anomalyButton').on('click', function (e) {
      e.preventDefault();
           graphsProperties = []
      for (let i = 0; i < ultraCharts.length; i++){
            graphsProperties.push(ultraCharts[i].data.datasets[0].data)
    }

      $.ajax({
          type: 'POST',
          url: '/anomaly_detection/',
          data: {
             csrfmiddlewaretoken: window.CSRF_TOKEN,
              graphInfo: JSON.stringify(graphsProperties),
              dataType: "json",
          },

          success: function (result) {
            parsedResults=[]
            // get results as a list of lists of indices and values
            resultDictionary = JSON.parse(result);

            // push each list to parsedResults to make an array of objects
              for (let i = 0; i < resultDictionary.length; i++){
              parsedResults.push(JSON.parse(resultDictionary[i]))
              }

            console.log( parsedResults)

            //1st loop over each list of objects
            //2nd loop over indices to color the points as outliers
           for (let i = 0; i < parsedResults.length; i++)  {
           for ( let j = 0; j< Object.keys(parsedResults[i]).length;j++){
            ultraCharts[i].getDatasetMeta(0).data[parseInt(Object.keys(parsedResults[i])[j])].custom = {
                backgroundColor:'lightgreen',
                borderColor:'lightgreen',
                 borderWidth: 4
            };
            }
            ultraCharts[i].update();
            }

          console.log("Success")
          },

          failure: function () {
          console.log("Failure")
          },

          complete: function (){

          },

      });
  });


  });

  if(!anomaly['AnomalyDetection']){
    document.getElementById("anomalyButton").remove();    
  }
