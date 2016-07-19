'use strict';
var utm17 = '+proj=utm +zone=17 +ellps=GRS80 +datum=NAD83 +units=m +no_defs';



angular.module('myApp.map')
.factory('geometryService', function() {
    function derivativeCurve() {
        var newPoints = [];
        var n = this.points.length;
        for(var i = 1; i < n; i++) {
            newPoints.push([n*(this.points[i][0] - this.points[i-1][0]),
                n*(this.points[i][1] - this.points[i-1][1])])
        }
        return new Bezier(newPoints);
    }

    function getBezierPoint(points, t) {
        if(points.length==1) {
            return points[0];
        } else if(points.length == 0) {
            return [0,0];
        }
        var newpoints = [];
        for(var i=0; i<points.length-1; i++) {
            var x = (1-t) * points[i][0] + t * points[i+1][0];
            var y = (1-t) * points[i][1] + t * points[i+1][1];
            newpoints.push([x,y]);
        }
        return getBezierPoint(newpoints, t);
    }

    function compute(t) {
        return getBezierPoint(this.points, t);
    }

/* The following code is adapted from https://github.com/Pomax/bezierjs, which is under an MIT license.
    // Legendre-Gauss abscissae with n=24 (x_i values, defined at i=n as the roots of the nth order Legendre polynomial Pn(x))
    var Tvalues=[
      -0.0640568928626056260850430826247450385909,
       0.0640568928626056260850430826247450385909,
      -0.1911188674736163091586398207570696318404,
       0.1911188674736163091586398207570696318404,
      -0.3150426796961633743867932913198102407864,
       0.3150426796961633743867932913198102407864,
      -0.4337935076260451384870842319133497124524,
       0.4337935076260451384870842319133497124524,
      -0.5454214713888395356583756172183723700107,
       0.5454214713888395356583756172183723700107,
      -0.6480936519369755692524957869107476266696,
       0.6480936519369755692524957869107476266696,
      -0.7401241915785543642438281030999784255232,
       0.7401241915785543642438281030999784255232,
      -0.8200019859739029219539498726697452080761,
       0.8200019859739029219539498726697452080761,
      -0.8864155270044010342131543419821967550873,
       0.8864155270044010342131543419821967550873,
      -0.9382745520027327585236490017087214496548,
       0.9382745520027327585236490017087214496548,
      -0.9747285559713094981983919930081690617411,
       0.9747285559713094981983919930081690617411,
      -0.9951872199970213601799974097007368118745,
       0.9951872199970213601799974097007368118745
    ];

    // Legendre-Gauss weights with n=24 (w_i values, defined by a function linked to in the Bezier primer article)
    var Cvalues = [
      0.1279381953467521569740561652246953718517,
      0.1279381953467521569740561652246953718517,
      0.1258374563468282961213753825111836887264,
      0.1258374563468282961213753825111836887264,
      0.1216704729278033912044631534762624256070,
      0.1216704729278033912044631534762624256070,
      0.1155056680537256013533444839067835598622,
      0.1155056680537256013533444839067835598622,
      0.1074442701159656347825773424466062227946,
      0.1074442701159656347825773424466062227946,
      0.0976186521041138882698806644642471544279,
      0.0976186521041138882698806644642471544279,
      0.0861901615319532759171852029837426671850,
      0.0861901615319532759171852029837426671850,
      0.0733464814110803057340336152531165181193,
      0.0733464814110803057340336152531165181193,
      0.0592985849154367807463677585001085845412,
      0.0592985849154367807463677585001085845412,
      0.0442774388174198061686027482113382288593,
      0.0442774388174198061686027482113382288593,
      0.0285313886289336631813078159518782864491,
      0.0285313886289336631813078159518782864491,
      0.0123412297999871995468056670700372915759,
      0.0123412297999871995468056670700372915759
    ];

    function arcfn(t, derivative) {
      var d = derivative.compute(t);
      var l = d[0]*d[0] + d[1]*d[1];

      return Math.sqrt(l);
    }
    function len() {
        var z=0.5,sum=0,len=Tvalues.length,i,t;
        var d = this.derivativeCurve();
        for(i=0; i<len; i++) {
            t = z * Tvalues[i] + z;
            sum += Cvalues[i] * arcfn(t,d);
        }
        return z * sum;
    }

    var Bezier = function(points) {
        this.points = points;
        this.compute = compute;
        this.derivativeCurve = derivativeCurve;
        this.len = len;
        this.LUT = []
        var t = numeric.linspace(0,1,200);
        for(var i = 0; i < t.length; i++) {
            this.LUT.push(getBezierPoint(points,t[i]));
        };
    };

    function calcDistance2(p1, p2) {
        return (p1[0]- p2[0])*(p1[0]- p2[0]) + (p1[1]- p2[1])*(p1[1]- p2[1]);
    }

    function nearestT(coord) {
        var minDist = 1e30;
        var minT = -1;
        var t = numeric.linspace(0,1,this.LUT.length);
        for(var i = 0; i < t.length; i++) {
            var dist = calcDistance2(this.LUT[i], coord);
            if(dist < minDist) {
                minDist = dist;
                minT = t[i]
            }
        }
        var interval = 1.0/this.LUT.length;
        var lastDiff = 1;
        while(lastDiff > 0.0001) {
            lastDiff = 1;
            var d1 = calcDistance2(this.compute(minT + interval/2.0), coord);
            var d2 = calcDistance2(this.compute(minT - interval/2.0), coord);
            if (d1 < minDist && d1 < d2) {
                lastDiff = minDist - d1;
                minDist = d1;
                minT += interval / 2.0;
            } else if (d2 < minDist && d2 < d1) {
                lastDiff = minDist - d2;
                minDist = d2;
                minT -= interval / 2.0;
            } else {
                interval /= 2.0;
                lastDiff = interval;
            }
        }
        return minT;
    }

    var VehiclePath = function(start, end, departureDir, receivingDir, d1, d2, midControls) {
        var p2x = start[0] + d1 * departureDir[0];
        var p2y = start[1] + d1 * departureDir[1];
        var p4x = end[0] + d2 * receivingDir[0];
        var p4y = end[1] + d2 * receivingDir[1];
        var coords = [start, [p2x, p2y]];
        for(var i = 0; i < midControls.length; i+=2) {
            coords.push([midControls[i], midControls[i+1]]);
        }
        coords.push([p4x,p4y], end);
        Bezier.call(this, coords);
        var dPath = this.derivativeCurve();
        var ddPath = dPath.derivativeCurve();
        var dddPath = ddPath.derivativeCurve();
        this.kLUT = [];
        this.dkdsLUT = [];
        this.computeK = function(t) {
            var d = dPath.compute(t);
            var dd = ddPath.compute(t);
            return (d[0] * dd[1] - d[1] * dd[0])/Math.pow(d[0] * d[0] + d[1] * d[1],1.5);
        };
        for (var i = 0; i < this.LUT.length; i++) {
            var d = dPath.LUT[i];
            var dd = ddPath.LUT[i];
            var ddd = dddPath.LUT[i];
            this.kLUT.push((d[0] * dd[1] - d[1] * dd[0])/Math.pow(d[0] * d[0] + d[1] * d[1],1.5));
            this.dkdsLUT.push((-3.0*d[0]*dd[0] - 3.0*d[1]*dd[1])*(d[0]*dd[1]- d[1]*dd[0])*Math.pow(d[0]*d[0] + d[1]*d[1], -2.5)
                +(d[0]*ddd[1] - d[1]*ddd[0])*Math.pow(d[0]*d[0] + d[1]*d[1], -1.5));
        }
        this.nearestT = nearestT;
    };

    var normalize = function(origin, dest) {
        var x = dest[0] - origin[0];
        var y = dest[1] - origin[1];
        var d = Math.sqrt(x*x+y*y);
        return [x/d, y/d];
    }
    var latLng2xy = function(latLng) {
        return proj4(utm17, [latLng.longitude, latLng.latitude])
    }
    var xy2latLng = function(xy) {
        var ll = proj4(utm17).inverse(xy);
        return {longitude: ll[0], latitude: ll[1]}
    }
    var normalizedAndCenterVectors = function (latLngPoints, latLngCenter) {
        var vectors = [];
        var tX = 0, tY = 0;
        var c = latLng2xy(latLngCenter);

        latLngPoints.forEach(function(point) {
            var p = latLng2xy(point);
            var v = normalize(c,p)
            vectors.push(v);
            tX += v[0];
            tY += v[1];
        });
        var d = Math.sqrt(tX*tX+tY*tY);

        return {
            vectors: vectors,
            centerVector: [tX/d, tY/d],
            origin: c
        };
    };

    var constructLaneLine = function(center, vector, offset, length, direction) {
        var p1 = [center[0] + offset * vector[0], center[1] + offset * vector[1]];
        var p2 = [p1[0] - length*vector[1]*direction, p1[1] + length*vector[0]*direction];
        p1 = xy2latLng(p1);
        p2 = xy2latLng(p2);
        return [p1, p2];
    }



    var turningRadius = function(wheelbase, maxSteeringAngle, width) {
        var coords = proj4(utm17, [-80.4820387, 43.4536148]);
        var odeEq = function(t,x) {
            var V = 1 /3.6;
            var d = maxSteeringAngle/180*Math.PI;
            var L = wheelbase;
            return [V*Math.cos(x[2]), -V*Math.sin(x[2]), V/L*Math.tan(d)];
        }
        var sol = numeric.dopri(0,60,[coords[0],coords[1],0],odeEq).at([5, 10, 15, 20, 30, 40, 50, 60]);
        var points = []
        for (var i = 0; i < sol.length; i++) {
            coords = proj4(utm17).inverse([sol[i][0]-wheelbase *Math.cos(sol[i][2]), sol[i][1] +wheelbase *Math.sin(sol[i][2])]);
            points.push({latitude: coords[1], longitude: coords[0], id: generateID()});
        }
        return points;//circleCompute(points).radius - (width/2.0);
    };

    function profile(curve, n1, start, vehicle) {
        var V = curve.len();//curve.len();
        var derCurve = curve.derivativeCurve();
        function odeEq(t,x) {
            var s = curve.nearestT(x.slice(0,2));
            var point = curve.compute(s)
            var tangent = derCurve.compute(s)
            var error = [point[0] - x[0], point[1]- x[1]];
            var dir = Math.sign(error[0]*tangent[1] - error[1]*tangent[0]);
            var derr = Math.atan2(tangent[1], tangent[0]) - x[2];

            var d = -Math.atan(curve.computeK(s)*vehicle.wheelbase);

            d -= derr * 1e-6;
            d += dir * Math.sqrt(calcDistance2(error, [0,0])) * 1e-1;
            return [
                V*Math.cos(x[2]),
                -V*Math.sin(x[2]),
                V/vehicle.wheelbase*Math.tan(d)
            ]
        }
        var sol = numeric.dopri(0,1,[start[0], start[1], Math.atan2(-n1[1], n1[0])],odeEq, 1e-6, 1000,
            function(t,x) {
                return curve.nearestT(x.slice(0,2)) -0.999;
            }
        );
        console.log(sol);
        var t = numeric.linspace(0,sol.x[sol.x.length-1],curve.LUT.length);
        var sol = sol.at(t);
        //console.log(sol);
        //console.log(vehicle);
        var points_rl = [];
        var points_rr = [];
        var points_fl = [];
        var points_fr = [];
        var front = vehicle.frontOverhang + vehicle.wheelbase;
        var points = []
        for(var i = 0; i < t.length; i++) {
            points.push([sol[i][0], sol[i][1]]);
            var rrx = sol[i][0] - vehicle.rearOverhang * Math.cos(sol[i][2]) - vehicle.width /2.0 * Math.sin(sol[i][2]);
            var rry = sol[i][1] + vehicle.rearOverhang * Math.sin(sol[i][2]) - vehicle.width /2.0 * Math.cos(sol[i][2]);
            points_rr.push([rrx,rry]);
            var rlx = sol[i][0] - vehicle.rearOverhang * Math.cos(sol[i][2]) + vehicle.width /2.0 * Math.sin(sol[i][2]);
            var rly = sol[i][1] + vehicle.rearOverhang * Math.sin(sol[i][2]) + vehicle.width /2.0 * Math.cos(sol[i][2]);
            points_rl.push([rlx,rly]);

            var flx = sol[i][0] + front * Math.cos(sol[i][2]) + vehicle.width /2.0 * Math.sin(sol[i][2]);
            var fly = sol[i][1] - front * Math.sin(sol[i][2]) + vehicle.width /2.0 * Math.cos(sol[i][2]);
            points_fl.push([flx,fly]);
            var frx = sol[i][0] + front * Math.cos(sol[i][2]) - vehicle.width /2.0 * Math.sin(sol[i][2]);
            var fry = sol[i][1] - front * Math.sin(sol[i][2]) - vehicle.width /2.0 * Math.cos(sol[i][2]);
            points_fr.push([frx,fry])
        }
        points_fl.reverse();
        points_fr.reverse();
        points_fl.push(points_rl[0]);
        points_fl.push(points_rr[0]);
        points_rr.push(points_fr[0]);
        console.log([points_rr.concat(points_fl)]);
        var points1 = {
            "id": 1,
            "type": "Feature",
            "properties": {},
            "geometry": {
                "type": "Polygon",
                "coordinates": [points_rr.concat(points_fl)]
            }
        }

        points_fr.push(points_rr[0]);
        points_fr.push(points_rl[0]);
        points_rl.push(points_fl[0]);
        var points2 = {
            "id": 2,
            "type": "Feature",
            "properties": {},
            "geometry": {
                "type": "Polygon",
                "coordinates": [points_rl.concat(points_fr)]
            }
        }
        points1 = turf.buffer(points1,1e-6,'km');
        points2 = turf.buffer(points2,1e-6,'km');
        console.log(points1);
        console.log(points2);
        var poly = turf.union(points1.features[0], points2.features[0]);
        console.log(poly);
        return poly.geometry.coordinates[0];
    }

    var turningRadiusTrailer = function(frontWheelbase, rearWheelbase, maxSteeringAngle, maxTrailerAngle, width) {
        var origin = proj4(utm17, [-80.4820387, 43.4536148]);
        var V = 10 /3.6;
        var d = maxSteeringAngle/180*Math.PI;
        var L = frontWheelbase;
        var T = rearWheelbase;
        var trailerLimit = maxTrailerAngle /180 *Math.PI;
        var odeEq = function(t,x) {
            var dg = -V/T * Math.sin(x[3]) - V/L*Math.tan(d);
            if ((x[3] > trailerLimit && dg > 0)  || (x[3] < - trailerLimit && dg < 0)) {
                dg = 0
            }
            return [V*Math.cos(x[2]), -V*Math.sin(x[2]), V/L*Math.tan(d), dg];
        }
        var sol = numeric.dopri(0,200,[0,0,0,0],odeEq, 1e-12, 10000).at(numeric.linspace(50,200));
        var points = []
        for (var i = 0; i < sol.length; i++) {
            var x = sol[i][0]  - T * Math.cos(sol[i][2] + sol[i][3]) - width/2.0 * Math.sin(sol[i][2] + sol[i][3])//- frontWheelbase *Math.cos(sol[i][2])//
            var y = sol[i][1]  + T * Math.sin(sol[i][2] + sol[i][3]) - width/2.0 * Math.cos(sol[i][2] + sol[i][3])//+ frontWheelbase *Math.sin(sol[i][2])//
            console.log(sol[i][3] * 180 / Math.PI);
            console.log(x,y);
            var coords = proj4(utm17).inverse([x + origin[0],y  + origin[1]]);
            points.push({latitude: coords[1], longitude: coords[0], id: generateID()});
        }
        return points; //circleCompute(points).radius;
    };

    var turningRadiusTrailer2 = function(frontWheelbase, rearWheelbase, maxSteeringAngle, maxTrailerAngle, width) {
        var origin = proj4(utm17, [-80.4820387, 43.4536148]);
        var V = 2/3.6;
        var d = maxSteeringAngle/180*Math.PI;
        var L = frontWheelbase;
        var T = rearWheelbase;
        var trailerLimit = maxTrailerAngle /180 *Math.PI;
        var odeEq = function(t,x) {
            var dg = V/T * Math.sin(x[2] - x[3]);
            var dth = V*Math.sin(x[2])*Math.cos(x[2] - x[3]);
            if (x[2] - x[3] > trailerLimit && dg < dth) {
                //dg = dth;
            } else if(x[2] - x[3] < -trailerLimit && dg > -dth) {
                //dg  = -dth;
            }
            return [V*Math.cos(x[2])*Math.cos(x[2] - x[3]), dth, V/L*Math.tan(d), dg];
        }
        var sol = numeric.dopri(0,100,[0,0,0,0],odeEq, 1e-12, 10000, function(t,x) { return x[2]-Math.PI;}).at(numeric.linspace(25,50));
        var points = []
        for (var i = 0; i < sol.length; i++) {
            var x = sol[i][0] + width/2.0 * Math.sin(sol[i][2] + sol[i][3])//- frontWheelbase *Math.cos(sol[i][2])//
            var y = sol[i][1]  - width/2.0 * Math.cos(sol[i][2] + sol[i][3])//+ frontWheelbase *Math.sin(sol[i][2])//
            console.log((sol[i][2] - sol[i][3]) * 180 / Math.PI,(sol[i][2]) * 180 / Math.PI, (sol[i][3]) * 180 / Math.PI );
            console.log(Math.cos(sol[i][2] - sol[i][3]));
            //console.log(x,y);
            var coords = proj4(utm17).inverse([x + origin[0],y  + origin[1]]);
            points.push({latitude: coords[1], longitude: coords[0], id: generateID()});
        }
        return points; //circleCompute(points).radius;
    };

    return {
        circleFromPoints: function(points) {
            // calculate best-fit cirlce
            var n = points.length;
            if (n < 3) { // not enough points
                return {
                    radius: undefined,
                    center: {latitude: 0, longitude: 0},
                    visible: false
                };
            }
            var x = [];
            var y = [];
            var cx = 0, cy = 0;

            // Centre points
            for(var i=0; i < n; i++) {
                var coords = latLng2xy(points[i]);
                x.push(coords[0]);
                y.push(coords[1]);
                cx += coords[0]/n;
                cy += coords[1]/n;
            }
            var XX = 0, YY = 0, XY = 0, XXX = 0, YYY = 0, XXY = 0, XYY = 0, YYY = 0;
            for(var i=0; i < n; i++) {
                x[i] -= cx;
                y[i] -= cy;
                XX += x[i] * x[i];
                YY += y[i] * y[i];
                XY += x[i] * y[i];
                XXX += x[i] * x[i] * x[i];
                XXY += x[i] * x[i] * y[i];
                XYY += x[i] * y[i] * y[i];
                YYY += y[i] * y[i] * y[i];
            }
            var u = 0.5 * (YY * (XXX + XYY) - XY * (YYY + XXY))/(XX * YY - XY * XY);
            var v = 0.5 * (XX * (YYY + XXY) - XY * (XXX + XYY))/(XX * YY - XY * XY);
            var R = Math.sqrt(u*u + v*v + (XX+YY)/n);
            var centre = xy2latLng([u+cx, v+cy]);
            return {
                radius: R,
                center: centre,
                visible: true
            };
        },
        makeOuterLaneLines: function(points, circle, width, length) {
            if (circle.radius === undefined) {
                // can't do anything yet
                return [];
            }
            //var width=3.5, length=20;
            var vectors = normalizedAndCenterVectors(points, circle.center);

            var maxXprod = Number.NEGATIVE_INFINITY;
            var minXprod = Number.POSITIVE_INFINITY;
            var maxVec, minVec;
            vectors.vectors.forEach(function(vec) {
                var xprod = vectors.centerVector[0]*vec[1] - vectors.centerVector[1]*vec[0];
                if (xprod > 0 && xprod > maxXprod) {
                    maxXprod = xprod;
                    maxVec = vec;
                } else if (xprod < 0 && xprod < minXprod) {
                    minXprod = xprod;
                    minVec = vec;
                }
            });
            var offset = circle.radius + width/2.0;

            return [
                constructLaneLine(vectors.origin, maxVec, offset, length, 1),
                constructLaneLine(vectors.origin, minVec, offset, length, -1)
            ];
        },
        initCurve: function(line1, line2, rightTurn, circle, vehicle) {
            var l1 = [latLng2xy(line1[0]), latLng2xy(line1[1])];
            var l2 = [latLng2xy(line2[0]), latLng2xy(line2[1])];

            var n1 = normalize(l1[1], l1[0]);
            var n2 = normalize(l2[1], l2[0]);
            var len1 = (l1[0][0] - l1[1][0])/n1[0];
            var len2 = (l2[0][0] - l2[1][0])/n2[0];
            var distance = 0;//vehicle.width/2.0 + 0.3;
            var dir = -1;
            if (rightTurn) {
                dir = 1;
            }
            var p1 = [l1[1][0] + dir * n1[1] * distance - n1[0] *0,
                      l1[1][1] - dir * n1[0] * distance - n1[1] *0];
            var p5 = [l2[1][0] - dir * n2[1] * distance - n2[0] *0,
                      l2[1][1] + dir * n2[0] * distance - n2[1] *0];
            var p = [5*circle.radius * (n1[0]+n2[0]), 5*circle.radius * (n1[1]+n2[1])];
            var parms = numeric.solve([[n1[0],-n2[0]],[n1[1],-n2[1]]],[p5[0]-p1[0],p5[1]-p1[1]])
            var p234 = [p1[0] + n1[0]*parms[0],
                        p1[1] + n1[1]*parms[0]];
            var p3 = [p234[0] - 15 * (n1[0] + n2[0]), p234[1] - 15 * (n1[1]+n2[1])];
            var controlPts = [p1, p234, p3, p234, p5];
            var ptArray = [];
            var c = latLng2xy(circle.center);
            //var d1 = Math.sqrt((p234[0]-p1[0])*(p234[0]-p1[0]) + (p234[1]-p1[1])*(p234[1]-p1[1]));
            //var d2 = Math.sqrt((p234[0]-p5[0])*(p234[0]-p5[0]) + (p234[1]-p5[1])*(p234[1]-p5[1]));
            //var b = new Bezier(controlPts);
            //var b = new VehiclePath(p1, p5, [n1[0], n1[1]], [n2[0], n2[1]], len1, len2, p3);
            function getPath(dist1, dist2, coords) {
                var coords2 = coords.slice();
                for (var i = 0; i < coords2.length; i+=2) {
                    coords2[i] += c[0];
                    coords2[i+1] += c[1];
                }
                return new VehiclePath(p1, p5, [n1[0], n1[1]], [n2[0], n2[1]], dist1, dist2, coords2);
            }
            function calcfc (N,M,X,CON) {
                var Y = X.slice(2)

                var path = getPath(X[0], X[1], Y);
                //console.log(path.points);
                var minDist = 1e9;
                for (var i = 0; i < path.LUT.length; i++) {
                    var dx = (path.LUT[i][0] - c[0]);
                    var dy = (path.LUT[i][1] - c[1])
                    var dist2 = dx*dx + dy*dy
                    if (dist2 < minDist) {
                        minDist = dist2;
                    }
                }
                CON[0] = Math.sqrt(minDist) - (circle.radius + vehicle.width/2.0 + 0.3);
                var maxCurve = 0;
                for (var i = 0; i< path.kLUT.length; i++) {
                    var curve = Math.abs(path.kLUT[i]);
                    if (curve > maxCurve) {
                        maxCurve = curve;
                    }
                }
                //CON[1] = 1.0/maxCurve - 10;
                CON[1] = vehicle.maxSteeringAngle - Math.abs(Math.atan(vehicle.wheelbase * maxCurve));
                CON[2] = X[0] - 2;
                CON[3] = X[1] - 2;
                CON[4] = (X[2] * (n1[0]+n2[0]) + X[3]* (n1[1]+n2[1]))*50;
                CON[5] = (X[4] * (n1[0]+n2[0]) + X[5]* (n1[1]+n2[1]))*50;

                return path.len();
            }
            var x = [len1/2, len2/2] //, p234[0] - c[0] + n1[0], p234[1] - c[1] + n1[1],
                //p234[0] - c[0], p234[1] - c[1],
                //p234[0] - c[0] + n2[0], p234[1] - c[1] + n2[1]];//p[0] ,p[1] ];//p3[0]- c[0], p234[1]- c[1], p234[0] - c[0], p234[1] - c[1]+1];
            for(var i = 0; i < 3; i++) {
                x.push(p234[0] - c[0] + (n1[0] + n2[0])*5);
                x.push(p234[1] - c[1] + (n1[1] + n2[1])*5);
            }
            var res = FindMinimum(calcfc, x.length,  6, x, 2, 1e-3, 0, 1000);
            var b = getPath(x[0], x[1], x.slice(2));

            var maxCurve = 0;
            for (var i = 0; i < b.LUT.length; i++) {
                ptArray.push(xy2latLng(b.LUT[i]));

                    var curve = Math.abs(b.kLUT[i]);
                    if (curve > maxCurve) {
                        maxCurve = curve;
                    }
            }
            console.log(maxCurve);
            var ctrlArray = [];
            for (var i = 0; i < b.points.length; i++) {
                ctrlArray.push(xy2latLng(b.points[i]));
            }

            console.log(Math.sqrt(0.3*127*(1/maxCurve)));
            var prof = profile(b, n1, p1, vehicle);
            var profArray = [];
            for (var i = 0; i < prof.length; i++) {
                profArray.push(xy2latLng(prof[i]));
            }
            return {path: ptArray, points: ctrlArray, sim: profArray};
        },
        bez: function(points) {
            var nPoints = [];
            for(var i = 0; i < points.length; i++) {
                nPoints.push(latLng2xy(points[i]));
            }
            var b = new Bezier(nPoints);
            var ptArray = [];
            for (var i = 0; i < b.LUT.length; i++) {
                ptArray.push(xy2latLng(b.LUT[i]));
            }
            return ptArray;
        },
        turningRadiusForVehicle: turningRadius,
        turningRadiusForTrailer: turningRadiusTrailer2
    };
});
