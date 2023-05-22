
  actuateDataFromQT = JSON.parse(actuateData12);  
  console.log(actuateDataFromQT);

  const socket = io('http://localhost:8000/web');
  console.log(socket)
  socket.on('connect', () => {
      console.log(`connect ${socket.id}`);
  });

  socket.on('disconnect', (data) => {
      console.log(`disconnect ${socket.id}`);
  });

  // preparing HTML canvas for video stream
  const image_elem = document.getElementById("streamer-image");
  const image_elem1 = document.getElementById("streamer-image1");
  const text_elem = document.getElementById("streamer-text");
  const canvas_od = document.getElementById("canvas_objects");
  const canvas_od1 = document.getElementById("canvas_objects1");
  var draw = canvas_od.getContext("2d");
  var draw1 = canvas_od1.getContext("2d");
  draw.font = "20px Arial";
  draw.lineWidth = 3;
  draw1.font = "20px Arial";
  draw1.lineWidth = 3;

  //actuation controls and sliders


//  var slider = document.getElementById("formControlRange");

//   slider.remove();

  var output = document.getElementById("output");


  // google Maps
  window.lat = 30;
  window.lng = 31;
  var map;
  var mark;
  var lineCoords = [];
  var initialize = function () {
      map = new google.maps.Map(document.getElementById('map-canvas'), { center: { lat: lat, lng: lng }, zoom: 12 });
      mark = new google.maps.Marker({ position: { lat: lat, lng: lng }, map: map });
  };
  window.initialize = initialize;
  document.querySelector('#action').addEventListener('click', function () {
      var text = document.getElementById("action").textContent;
      if (text == "Start Tracking") {
          document.getElementById("action").classList.add('btn-danger');
          document.getElementById("action").classList.remove('btn-primary');
          document.getElementById("action").textContent = 'Stop Tracking';

      }
      else {
          document.getElementById("action").classList.remove('btn-danger');
          document.getElementById("action").classList.add('btn-primary');
          document.getElementById("action").textContent = 'Start Tracking';
      }
  });

  //loading model then allow acceptaance of socket events
  //console.log(canvas_od.width, canvas_od.height)
  cocoSsd.load().then(model => {
      socket.on('server2webvirtual', (a) => {
          image_elem.src = a.image;
          var date = new Date();
          var timestamp = date.getTime();

          var difference = timestamp - a["upload_time"]*1000;

          console.log('Received image in (digital): ', difference);
  
          image_elem.onload = () => {
              draw.drawImage(image_elem, 0, 0, canvas_od.width, canvas_od.height);
          };
          delay(1).then(delayers => model.detect(image_elem).then(predictions => {
              for (i = 0; i < predictions.length; i++) {
                  // get bounding box and label
                  const box = predictions[i].bbox;
                  const label = predictions[i].class;

                  // mark objects of different types with different colors
                  var color_code = "#FFA500";
                  switch (label) {
                      case "person":
                          color_code = "#4682B4";
                          break;

                      case "horse":
                          color_code = "#DB7093";
                          break;

                      case "cat":
                          color_code = "#C71585";
                          break;

                      case "dog":
                          color_code = "#BA55D3";
                          break;

                      case "car":
                          color_code = "#008080";
                          break;

                      case "bus":
                          color_code = "#9ACD32";
                          break;

                      case "truck":
                          color_code = "#40E0D0";
                          break;

                      case "airplane":
                          color_code = "#32CD32";
                          break;

                      case "boat":
                          color_code = "#808000";
                          break;

                      case "train":
                          color_code = "#228B22";
                          break;

                      // add more classes here as necessary

                  }

                  // draw on canvas

                  draw.fillStyle = color_code;
                  draw.strokeStyle = color_code;

                  draw.strokeRect(box[0], box[1], box[2], box[3]);
                  draw.fillRect(box[0], box[1] - 30, 70, 30);
                  draw.fillStyle = "white";
                  draw.fillText(label, box[0], box[1] - 5);

              }
          }));

          var date = new Date();
          var timestampafterinference = date.getTime();

          var difference = timestampafterinference - timestamp;

          console.log('Virtual Inference time: ', difference);

      });
  });

  cocoSsd.load().then(model => {
    socket.on('server2webphysical', (a) => {
        image_elem1.src = a.image;
        var date = new Date();
        var timestamp = date.getTime();

        var difference = timestamp - a["upload_time"]*1000;

        console.log('Received image in (physical): ', difference);
        image_elem1.onload = () => {
            draw1.drawImage(image_elem1, 0, 0, canvas_od1.width, canvas_od1.height);
        };
        delay(1).then(delayers => model.detect(image_elem1).then(predictions => {
            for (i = 0; i < predictions.length; i++) {
                // get bounding box and label
                const box = predictions[i].bbox;
                const label = predictions[i].class;

                // mark objects of different types with different colors
                var color_code = "#FFA500";
                switch (label) {
                    case "person":
                        color_code = "#4682B4";
                        break;

                    case "horse":
                        color_code = "#DB7093";
                        break;

                    case "cat":
                        color_code = "#C71585";
                        break;

                    case "dog":
                        color_code = "#BA55D3";
                        break;

                    case "car":
                        color_code = "#008080";
                        break;

                    case "bus":
                        color_code = "#9ACD32";
                        break;

                    case "truck":
                        color_code = "#40E0D0";
                        break;

                    case "airplane":
                        color_code = "#32CD32";
                        break;

                    case "boat":
                        color_code = "#808000";
                        break;

                    case "train":
                        color_code = "#228B22";
                        break;

                    // add more classes here as necessary

                }

                // draw on canvas

                draw1.fillStyle = color_code;
                draw1.strokeStyle = color_code;

                draw1.strokeRect(box[0], box[1], box[2], box[3]);
                draw1.fillRect(box[0], box[1] - 30, 70, 30);
                draw1.fillStyle = "white";
                draw1.fillText(label, box[0], box[1] - 5);
                var date = new Date();
                var timestampafter = date.getTime();
        
                var difference = timestampafter - timestamp;
        
                console.log('Physical Inference time: ', difference);
            }
        }));

    });
});
  //delay to allow drawing
  async function delay(delayInms) {
      return new Promise(resolve => {
          setTimeout(() => {
              resolve(2);
          }, delayInms);
      });
  }


//   GPS

  //real time data recieve 
  socket.on('sensedData', (data) => {
      console.log(data);

    if(data["sensor_name"]=="Position"){

        map.setCenter({ lat: data["lat"], lng: data["lng"], alt: 0 });
        mark.setPosition({ lat: data["lat"], lng: data["lng"], alt: 0 });

        lineCoords.push(new google.maps.LatLng(data["lat"], data["lng"]));

        var lineCoordinatesPath = new google.maps.Polyline({
            path: lineCoords,
            geodesic: true,
            strokeColor: '#2E10FF'
        });

        lineCoordinatesPath.setMap(map);

    }

  });


  var streaming = document.getElementById("streaming");
  var gpsTracking = document.getElementById("gpsTracking");
  var actuateDev = document.getElementById("actuateDev");

  console.log(actuateDataFromQT['Camera']);
  console.log(actuateDataFromQT['GPS']);
  console.log(actuateDataFromQT['Actuate']);

  if(!actuateDataFromQT['Camera'])
  {
    streaming.remove();
  }

  if(!actuateDataFromQT['GPS'])
  {
    gpsTracking.remove();
  }

  if(!actuateDataFromQT['Actuate'])
  {
    actuateDev.remove();
  }

//two implementations for teleop navigation

//  actuate_dict = {'w':0,'a':0,'s':0,'d':0,'x':0}
  actuate_dict ={}

   function direction(clicked_id) {
//       clicked_id == 1 ? 0 : 1
//timestamp here --------------------------------
      var timestamp = getTime()
      console.log(timestamp);
      socket.emit('new_data', 's');
  }
$(document).keydown(function(e) {
  if (e.which==37 ||e.which==65 ) {
    $('.left').addClass('pressed');
    $('.lefttext').text('LEFT');
    $('.left').css('transform', 'translate(0, 2px)');
//     socket.emit('new_data', JSON.stringify(actuate_dict));
     socket.emit('new_data', 'a');
  } else if (e.which==38 ||e.which==87) {
    $('.up').addClass('pressed');
    $('.uptext').text('UP');
    $('.left').css('transform', 'translate(0, 2px)');
    $('.down').css('transform', 'translate(0, 2px)');
    $('.right').css('transform', 'translate(0, 2px)');
//     socket.emit('new_data', JSON.stringify(actuate_dict));
     socket.emit('new_data', 'w');
  } else if (e.which==39 ||e.which==68) {
    $('.right').addClass('pressed');
    $('.righttext').text('RIGHT');
    $('.right').css('transform', 'translate(0, 2px)');
     socket.emit('new_data','d');
  } else if (e.which==40||e.which==83) {
    $('.down').addClass('pressed');
    $('.downtext').text('DOWN');
    $('.down').css('transform', 'translate(0, 2px)');
    socket.emit('new_data', 'x');
  }
});

$(document).keyup(function(e) {
  if (e.which==37 ||e.which==65 ) {
    $('.left').removeClass('pressed');
    $('.lefttext').text('');
    $('.left').css('transform', 'translate(0, 0)');

  } else if (e.which==38 ||e.which==87)  {
    $('.up').removeClass('pressed');
    $('.uptext').text('');
    $('.left').css('transform', 'translate(0, 0)');
    $('.down').css('transform', 'translate(0, 0)');
    $('.right').css('transform', 'translate(0, 0)');

  } else if (e.which==39 ||e.which==68) {
    $('.right').removeClass('pressed');
    $('.righttext').text('');
    $('.right').css('transform', 'translate(0, 0)');

  } else if (e.which==40||e.which==83)  {
    $('.down').removeClass('pressed');
    $('.downtext').text('');
    $('.down').css('transform', 'translate(0, 0)');

  }
});