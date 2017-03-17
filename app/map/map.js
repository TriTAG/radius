'use strict';

angular.module('myApp.map', ['ngRoute', 'ngMaterial'])

.config(['$routeProvider', '$mdThemingProvider', '$mdIconProvider', function($routeProvider,$mdThemingProvider, $mdIconProvider) {
    $routeProvider.when('/map/:points?', {
        templateUrl: 'map/map.html',
        controller: 'cornerCtrl'
    });

    $mdThemingProvider.theme('default')
    .primaryPalette('light-green')
    .accentPalette('red');
    /*$mdIconProvider
    .defaultIconSet('img/icons/sets/social-icons.svg', 24);*/
}])
.controller('cornerCtrl', ['$scope', '$route', '$routeParams', '$timeout', '$mdColors', 'geometryService', 'uiGmapGoogleMapApi',
function($scope, $route, $routeParams, $timeout, $mdColors, geometryService, uiGmapGoogleMapApi) {

    uiGmapGoogleMapApi.then(function(maps) {
        if ($routeParams.points) {
            $scope.points = google.maps.geometry.encoding.decodePath($routeParams.points)
            .map(function(val) {
                return {
                    latitude: val.lat(),
                    longitude: val.lng(),
                    id: generateID()
                };
            });
        } else {
            $scope.points = [];
        }
        $scope.vehicles = [
            {
                name: 'Car',
                width: 2.13, //2.59,
                wheelbase: 3.35,//7.62,
                frontOverhang: .91,//2.13,
                rearOverhang: 1.52,//2.44,
                maxSteeringAngle: Math.PI/180.0 *31.2,//Math.PI* 41.0/180.0,
                enabled: true
            },
            {
                name: 'Delivery truck',
                width: 2.59, //2.59,
                wheelbase: 3.95,
                frontOverhang: 0.96,
                rearOverhang: 1.98,
                maxSteeringAngle: Math.PI* 31.7/180.0,
                enabled: true
            },
            {
                name: 'Bus',
                width: 2.59, //2.59,
                wheelbase: 7.62,
                frontOverhang: 2.13,
                rearOverhang: 2.44,
                maxSteeringAngle: Math.PI* 41.0/180.0,
                enabled: true
            },
            {
                name: 'Single unit truck',
                width: 2.59,
                wheelbase: 6.1,
                frontOverhang: 1.22,
                rearOverhang: 1.83,
                maxSteeringAngle: Math.PI* 31.7/180.0,
                enabled: true
            },
            {
                name: 'Fire truck',
                width: 2.59, //2.59,
                wheelbase: 6.096,//7.62,
                frontOverhang: 2.59,//2.13,
                rearOverhang: 4.01,//2.44,
                maxSteeringAngle: Math.PI/180.0 *40,
                enabled: true//Math.PI* 41.0/180.0,
            },
            {
                name: 'Tractor trailer',
                width: 2.59, //2.59,
                wheelbase: 6.096,//7.62,
                frontOverhang: 2.59,//2.13,
                rearOverhang: 4.01,//2.44,
                maxSteeringAngle: Math.PI/180.0 *40,
                enabled: false//Math.PI* 41.0/180.0,
            }
        ];
        $scope.selectedVehicle = $scope.vehicles[0];
        $scope.corner = {
            right: true
        };
        $scope.points2 = [
            { latitude: 43.4536148, longitude: -80.4820387 },
            { latitude: 43.4537148, longitude: -80.4820387 },
        ];

        //$scope.points = geometryService.turningRadiusForTrailer(6.6,12.34, 28.4,65, 2.59);//15.24,
        $scope.circleDisplay = {
            stroke: {
                color: $mdColors.getThemeColor('light-green-500'),
                weight: 2,
                opacity: 1
            },
            fill: {
                color: $mdColors.getThemeColor('light-green-500'),
                opacity: 0.5
            },
            clickable: false
        };
        $scope.road = 0;
        $scope.map = {
            center: { latitude: 43.4536148, longitude: -80.4820387 },
            zoom: 20,
            options: {
                mapTypeId: google.maps.MapTypeId.SATELLITE,
                streetViewControl: false,
                mapTypeControl: false,
                draggableCursor: 'default'
            },
            control: {},
            events: {
                click: function (mapModel, eventName, originalEventArgs) {
                    var e = originalEventArgs[0];
                    var latLng = {
                        latitude: e.latLng.lat(),
                        longitude: e.latLng.lng(),
                        id: generateID()
                    };
                    $scope.points.push(latLng);

                    $scope.$apply();
                }
            }
        };


        $scope.markerOptions = {
            draggable: true,
        };
        $scope.markerEvents = {
            dblclick: function (marker, eventName, model, args) {
                var i = $scope.points.findIndex(function(m) { return m.id == model.id; });
                $scope.points.splice(i, 1);
                $scope.$apply();
            }
        }
        $scope.nodeIcon = {
            path: 'M -1,-1 1,-1, 1,1 -1,1 z',
            strokeOpacity: 0.7,
            strokeColor: $mdColors.getThemeColor('red-A700'),
            fillOpacity: 0.3,
            fillColor: 'white',
            scale: 2
        };

        $scope.departure = {
            outer: new OuterLine('white')
        };
        $scope.receiving = {
            outer: new OuterLine('blue')
        };
        $scope.path = {
            coords: [],
            start: [],
            end: [],
            stroke: {
                opacity: 0.75,
                color: 'orange'
            },
            fill: {
                opacity: 0.5,
                color: 'orange'
            },
            car_stroke: {
                opacity: 0.75,
                color: 'red'
            },
            car_fill: {
                opacity: 0.5,
                color: 'red'
            }
        };
        $scope.control = {coords: []};

        //$scope.circle = { center: null, radius: 0, visible: false };
        $scope.$watch(
            'points',
            function(nv, ov) {
                $scope.circle = geometryService.circleFromPoints($scope.points);
                var coords = geometryService.makeOuterLaneLines($scope.points, $scope.circle, 3.5, 20);

                if (coords.length > 0) {
                    if ($scope.corner.right) {
                        if (! $scope.departure.outer.edited) {
                            $scope.departure.outer.coords = coords[0];
                        }
                        if (! $scope.receiving.outer.edited) {
                            $scope.receiving.outer.coords = coords[1];
                        }
                    } else {
                        if (! $scope.departure.outer.edited) {
                            $scope.departure.outer.coords = coords[1];
                        }
                        if (! $scope.receiving.outer.edited) {
                            $scope.receiving.outer.coords = coords[0];
                        }
                    }

                } else {
                    $scope.departure.outer.coords = [];
                    $scope.departure.outer.edited = false;
                    $scope.receiving.outer.coords = [];
                    $scope.receiving.outer.edited = false;
                }
                var speed = Math.sqrt(0.4*127*($scope.circle.radius+1.5));
                $scope.speed = speed.toFixed(0) + ' km/h';
                $scope.risk = (100.0/(1.0 + Math.exp(6.9 - 0.090*speed))).toFixed(1) + ' %';
            }, true
        );
        $scope.$watch(
            'corner.right',
            function(nv, ov) {
                var temp = $scope.departure.outer.coords;
                $scope.departure.outer.coords = $scope.receiving.outer.coords;
                $scope.receiving.outer.coords = temp;
            }
        );
        var coordChange = function() {
            if($scope.departure.outer.coords.length > 1) {
                /*var vehicle = {
                    width: 2.13, //2.59,
                    wheelbase: 3.35,//7.62,
                    frontOverhang: .91,//2.13,
                    rearOverhang: 1.52,//2.44,
                    maxSteeringAngle: Math.PI/180.0 *31.2//Math.PI* 41.0/180.0,

                }*/
                // var vehicle = { // firetruck
                //     width: 2.59, //2.59,
                //     wheelbase: 6.096,//7.62,
                //     frontOverhang: 2.59,//2.13,
                //     rearOverhang: 4.01,//2.44,
                //     maxSteeringAngle: Math.PI/180.0 *40//Math.PI* 41.0/180.0,
                //
                // }

                var curve = geometryService.initCurve($scope.departure.outer.coords,
                    $scope.receiving.outer.coords, $scope.corner.right, $scope.circle, $scope.selectedVehicle);
                $scope.path.coords = curve.sim;
                $scope.path.start = curve.start;
                $scope.path.end = curve.end;
                $scope.control.coords = curve.points;
            }
        }
        $scope.$watch('departure.outer.coords', coordChange, true);
        $scope.$watch('receiving.outer.coords', coordChange, true);
        $scope.$watch('selectedVehicle', coordChange, true);
        $scope.$watch('control.coords', function() {
            //$scope.path.coords = geometryService.bez($scope.control.coords);
        }, true);
    });
}]);

var ALPHABET = '0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ';

var ID_LENGTH = 8;

var generateID = function() {
    var rtn = '';
    for (var i = 0; i < ID_LENGTH; i++) {
        rtn += ALPHABET.charAt(Math.floor(Math.random() * ALPHABET.length));
    }
    return rtn;
};

function OuterLine(color) {
    this.coords = [];
    this.edited = false;
    this.options = {
        draggable: true,
        visible: false
    };
    this.stroke = {
        color: color
    };
    this.events = {
        mousedown: function(polyline, eventName, model, args) {
            if (typeof args[0].vertex === 'undefined') {
                polyline.setEditable(false);
                polyline.setEditable(true);
            }
        },
        mouseup: function(polyline, eventName, model, args) {
            if (typeof args[0].vertex != 'undefined') {
                this.edited = true;
            }
        }.bind(this)
    }
}
