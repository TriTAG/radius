'use strict';

// Declare app level module which depends on views, and components
angular.module('myApp', [
  'ngRoute',
  'ngMaterial',
  'myApp.map',
  'myApp.version',
  'uiGmapgoogle-maps'
]).
config(['$routeProvider', function($routeProvider) {
  $routeProvider.otherwise({redirectTo: '/map'});
}])
.config(function(uiGmapGoogleMapApiProvider) {
    uiGmapGoogleMapApiProvider.configure({
        //    key: 'your api key',
        key: 'AIzaSyA-btJ3-kvPpUBL6tX4eYihoBN76VG_yp4',
        v: '3.23', //defaults to latest 3.X anyhow
        libraries: 'geometry'
    });
});
