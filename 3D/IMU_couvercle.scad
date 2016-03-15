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
diamVisCouv             = 2;

fn = 100;

difference() {
    
    cube(size = [baseLength, baseWidth,  epaisseur]);
    translate([epaisseur/2, epaisseur / 2, -0.1])
            cylinder(h = epaisseur + 0.2, d = diamVisCouv, $fn = fn);
    
    translate([epaisseur/2, baseWidth - epaisseur / 2, -0.1])
           cylinder(h = epaisseur + 0.2, d = diamVisCouv, $fn = fn);
    
    translate([baseLength - epaisseur, 0, 0]) {
        translate([epaisseur/2, epaisseur / 2, -0.1])
            cylinder(h = epaisseur + 0.2, d = diamVisCouv, $fn = fn);
    
        translate([epaisseur/2, baseWidth - epaisseur / 2, -0.1])
                cylinder(h = epaisseur + 0.2, d = diamVisCouv, $fn = fn);
    }
    
}
    
translate([0, baseWidth / 2, hauteurBoite - filDiam])
    rotate([180, 0, 0])
        difference() {
            translate([0, -filDiam / 2, 0])
                cube(size = [epaisseur, filDiam, hauteurBoite - epaisseur - filDiam]);
            translate([-0.1,0,0])
            rotate([0, 90, 0])
                cylinder(h = epaisseur + 0.2, d = filDiam, $fn = fn);
            
        }
        
