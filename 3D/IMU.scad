/* 
 * Copyright (C) 2015  TELECOM Nancy
 *
 * This model is under the Creative Common CC-BY-SA-NC license.
 *
 */

boardLength             = 22.5;
boardWidth              = 19.5;
boardHeight             = 4.5;

boardGap                = 5;

screwHoleDiam           = 3;
screwSpaceFromEdge      = 1;
screwEp                 = 2;
screwDiam               = 2;

fixationLength          = 20;

epaisseur               = 3;
hauteurBoite            = 25;
filDiam                 = 4;


elevationCarte          = hauteurBoite - boardGap - epaisseur;
baseLength              = boardLength + 2 * (boardGap + epaisseur);
baseWidth               = boardWidth + 2 * (boardGap + epaisseur);

screwCylDiff            = epaisseur + boardGap + screwSpaceFromEdge + screwHoleDiam/2;

diamVisFix              = 4;

fn = 100;

cube(size = [baseLength, baseWidth,  epaisseur]);

//triangle haut
translate([0, baseWidth, 0])
    difference() {
            polyhedron(
                points = [
                          [0, 0, 0], [baseLength/2, fixationLength,0], [baseLength, 0, 0],
                          [0, 0, epaisseur], [baseLength/2, fixationLength,epaisseur], [baseLength, 0, epaisseur]
                         ],
                faces = [
                          [0, 2, 1], [3, 4, 5], [0, 1, 4,3],
                          [0, 3, 5, 2], [1, 2, 5, 4]
                        ]
            );
            translate([baseLength / 2, fixationLength / 2.5, -0.1])
                cylinder(h = epaisseur + 0.2, d = diamVisFix, $fn = fn);
        }
    
 //triangle bas
rotate([180, 0, 0])
    translate([0, 0, -epaisseur])
        difference() {
            polyhedron(
                points = [
                          [0, 0, 0], [baseLength/2, fixationLength,0], [baseLength, 0, 0],
                          [0, 0, epaisseur], [baseLength/2, fixationLength,epaisseur], [baseLength, 0, epaisseur]
                         ],
                faces = [
                          [0, 2, 1], [3, 4, 5], [0, 1, 4,3],
                          [0, 3, 5, 2], [1, 2, 5, 4]
                        ]
            );
            translate([baseLength / 2, fixationLength / 2.5, -0.1])
                cylinder(h = epaisseur + 0.2, d = diamVisFix, $fn = fn);
        }
//face gauche
difference() {
    cube(size = [epaisseur, baseWidth, hauteurBoite]);
    
    translate([epaisseur/2, epaisseur / 2, 0.1])
            cylinder(h = hauteurBoite, d = 1, $fn = fn);
    
    translate([epaisseur/2, baseWidth - epaisseur / 2, 0.1])
            cylinder(h = hauteurBoite, d = 1, $fn = fn);
    
    translate([-0.1, baseWidth / 2, epaisseur + filDiam])
        union() {
            translate([0, -filDiam / 2, 0])
                cube(size = [epaisseur + 0.2, filDiam, hauteurBoite - epaisseur - filDiam + .01]);
            rotate([0, 90, 0])
                cylinder(h = epaisseur + 0.2, d = filDiam, $fn = fn);
            
        }
    
}
//face droite
translate([baseLength - epaisseur, 0, 0]) {
    difference() {
        cube(size = [epaisseur, baseWidth, hauteurBoite]);
         translate([epaisseur/2, epaisseur / 2, 0.1])
            cylinder(h = hauteurBoite, d = 1, $fn = fn);
    
        translate([epaisseur/2, baseWidth - epaisseur / 2, 0.1])
                cylinder(h = hauteurBoite, d = 1, $fn = fn);
    }
}

    
//face bas
translate([epaisseur, 0, 0])
    cube(size = [baseLength - 2 * epaisseur, epaisseur, hauteurBoite]);

//face haut
translate([epaisseur, baseWidth - epaisseur, 0])
    cube(size = [baseLength - 2 * epaisseur, epaisseur, hauteurBoite]);

//Bas gauche
translate( [ screwCylDiff, screwCylDiff, epaisseur])
    difference () {
        cylinder(h = elevationCarte, d = screwHoleDiam + screwEp, $fn = fn);
        cylinder(h = elevationCarte + 0.1, d = screwDiam, $fn = fn);
    }

//bas droite
translate( [ baseLength - screwCylDiff, screwCylDiff, epaisseur])
    difference () {
        cylinder(h = elevationCarte, d = screwHoleDiam + screwEp, $fn = fn);
        cylinder(h = elevationCarte + 0.1, d = screwDiam, $fn = fn);
    }

// haut droite
translate( [ baseLength - screwCylDiff, baseWidth - screwCylDiff, epaisseur])
    difference () {
        cylinder(h = elevationCarte, d = screwHoleDiam + screwEp, $fn = fn);
        cylinder(h = elevationCarte + 0.1, d = screwDiam, $fn = fn);
    }
 
 // haut gauche
translate( [ screwCylDiff, baseWidth - screwCylDiff, epaisseur])
    difference () {
        cylinder(h = elevationCarte, d = screwHoleDiam + screwEp, $fn = fn);
        cylinder(h = elevationCarte + 0.1, d = screwDiam, $fn = fn);
    }
