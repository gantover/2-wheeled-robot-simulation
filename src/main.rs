use plotters::prelude::*;
use itertools_num::linspace;
use std::f64::consts::PI;
//use std::thread; // essai de multithreading
//use crossbeam;

const DT: f64 = 0.005;
// Vehicule
const L_BARRE_ROUES: f64 = 0.095; // 0.14 pour v3
const L_ROUES: f64 = 0.12;
const RAYON_ROUES: f64 = 0.03;
const VAL_BARRE_8: [isize; 8] = [-3500,-2500,-1500,-500,500,1500,2500,3500];
const BARRE_8: [f64; 8] = [-33.3375,-23.8125,-14.2875,-4.7625,4.7625,14.2875,23.8125,33.3375];
const VAL_BARRE_3: [isize; 3] = [-2400,0,2400];
const BARRE_3: [f64; 3] = [-21.,0.,21.];
const POS_REL_CAP_LAT: Position = Position {x: -0.1, y: 0.05};
const DEMI_L_BALISE: f64 = 0.05;
const DEMI_H_BALISE: f64 = 0.017/2.;
// Moteur
const PW_MIN: f64 = 12.;
const TENSION_MAX: f64 = 7.5;
const CARRAC_A: f64 = 3.932;
const CARRAC_B: f64 = -0.2163;
const BASE_SPEED_L: f64 = 255.;
const BASE_SPEED_R: f64 = 255.;
const TOP_SPEED_L: f64 = 255.;
const TOP_SPEED_R: f64 = 255.;
// Map
const MARGE: f64 = 0.017 / 2.;

#[derive(Clone)]
struct ResultatSimulation {
    centre_roues: Vec<Position>,
    centre_capeurs: Vec<Position>,
    hauteur: f64,
    kp_value: f64,
    cap_lat: Vec<Position>,
    var_omega_tot: f64,
    var_omega_vec: Vec<Position>,
}

fn simulation(&kp_value: &f64, line1: &Line, line2: &Line, line3: &Line, curve1: &Curve, curve2: &Curve, balise1: &Balise, balise2: &Balise, balise3: &Balise, balise4: &Balise) -> Option<ResultatSimulation> {
    let val_barre = VAL_BARRE_8;
    let mut car = Vehicule {omega: 0.1, vitesse: 0., theta: PI/2., position: Position { x: 0.03, y: 0. }, position_centre_capteurs: Position { x: 0., y: 0. }, position_capteurs: Vec::new(), position_capteur_lat: Position { x: 0., y: 0. }};
    car.update_position();
    car.get_position_capteurs();
    let mut moteur = Moteur {kp: kp_value, kd: 0.1, v_l: 0., v_r: 1., top_speed_l: TOP_SPEED_L, top_speed_r: TOP_SPEED_R, default_kp: kp_value};
    let mut temps: f64 = 0.;
    let mut centre_capeurs: Vec<Position> = Vec::new();
    let mut centre_roues: Vec<Position> = Vec::new();
    let mut cap_lat: Vec<Position> = Vec::new();
    let mut times_touched: usize;
    let mut error: isize = 0;
    let mut poss_error: isize;
    let mut var_omega_tot: f64 = 0.;
    let mut last_error = 0;
    let mut delta_error;
    let mut var_omega_vec: Vec<Position> = Vec::new();
    let mut abs_car_omega: f64;

    loop {
        times_touched = 0;
        for id_capteur in 0..8 {
            for line in [&line1, &line2, &line3] {
                if line.check_contact(&car.position_capteurs[id_capteur]) {
                    poss_error = val_barre[id_capteur];
                    times_touched += 1;
                    if times_touched > 1 {
                        error = error + poss_error;
                    } else {
                        error = poss_error;
                    }
                }
            }
            for curve in [&curve1, &curve2] {
                if curve.check_contact(&car.position_capteurs[id_capteur]) {
                    poss_error = val_barre[id_capteur];
                    times_touched += 1;
                    if times_touched > 1 {
                        error = error + poss_error
                    } else {
                        error = poss_error;
                    }
                }
            }
        }
        // Module pour prendre en compte les balises
        //for balise in [&balise1, &balise2, &balise3, &balise4] {
        //    if balise.part.is_in_zone(&car.position_capteur_lat) {
                //moteur.update_top_speed(balise.id)
        //    }
        //}
        if times_touched > 1 {
            error = error / times_touched as isize;
        }
        delta_error = error - last_error;
        last_error = error;
        moteur.control(error,delta_error);
        car.omega = (RAYON_ROUES / L_ROUES) * (moteur.v_r - moteur.v_l);
        var_omega_tot += car.omega.abs();
        abs_car_omega = car.omega.abs();
        var_omega_vec.push(Position{ x:temps, y:abs_car_omega });
        car.vitesse = (RAYON_ROUES / 2.) * (moteur.v_r + moteur.v_l);
        car.update_position();
        car.get_position_capteurs();
        centre_roues.push(car.position);
        centre_capeurs.push(car.position_centre_capteurs);
        cap_lat.push(car.position_capteur_lat);
        temps += DT;
        if temps > 10. {
            break;
        }
    }
    if car.position.x > 0.5 && car.position.y > 0.5 && var_omega_tot > 100. { // qualification
        let hauteur = car.position.y;
        return Some(ResultatSimulation { centre_roues, centre_capeurs, hauteur, kp_value, cap_lat, var_omega_tot, var_omega_vec })
    } else {
        None
    }
}

fn main() {
    // def plotter 
    let root_area = BitMapBackend::new("path.png",(1200,2000)).into_drawing_area();
    root_area.fill(&WHITE).unwrap();
    let mut ctx = ChartBuilder::on(&root_area)
        .margin(10)
        .set_label_area_size(LabelAreaPosition::Left, 40)
        .set_label_area_size(LabelAreaPosition::Bottom, 40)
        .build_cartesian_2d(-0.2..1.0, 0.0..2.6)
        .unwrap();
    ctx.configure_mesh().draw().unwrap();
    // def second plotter (omega var)
    let root_area_2 = BitMapBackend::new("omega.png",(2000,1200)).into_drawing_area();
    root_area_2.fill(&WHITE).unwrap();
    let mut ctx_2 = ChartBuilder::on(&root_area_2)
        .margin(10)
        .set_label_area_size(LabelAreaPosition::Left, 40)
        .set_label_area_size(LabelAreaPosition::Bottom, 40)
        .build_cartesian_2d(0.0..10.0, 0.0..8.0)
        .unwrap();
    ctx_2.configure_mesh().draw().unwrap();

    // def map 
    let line1 = Line {part: Part {zone_x: [-0.2, 0.1], zone_y: [0., 2.2]}, start: Position {x: 0., y: 0.}, end: Position {x: 0. , y: 2.2} };
    let line2 = Line {part: Part {zone_x: [0.1, 0.3], zone_y: [0.4, 2.2]}, start: Position {x: 0.2, y: 0.4}, end: Position {x: 0.2 , y: 2.2} };
    let line3 = Line {part: Part {zone_x: [0.6, 1.], zone_y: [0.4, 2.25]}, start: Position {x: 0.8, y: 0.4}, end: Position {x: 0.8 , y: 2.25} };
    let lines = vec![&line1, &line2, &line3];
    let blue = &BLUE;
    let red = &RED;
    let green = &GREEN;
    let yellow = &YELLOW;
    let circuit_stroke_size = 6;
    for &line in lines {
        ctx.draw_series(
            LineSeries::new(
                line.get_drawing_data().iter().map(|v| {(v.x, v.y)}),
                yellow.stroke_width(circuit_stroke_size),
                )
            ).unwrap();
    }
    let curve1 = Curve {part: Part { zone_x: [-0.2, 0.4], zone_y: [2.2, 3.] }, origine: Position { x: 0., y: 2.2 }, radius: 0.1, direction: 1, orientation: 1 };
    let curve2 = Curve {part: Part { zone_x: [0.1, 0.9], zone_y: [0., 0.4] }, origine: Position { x: 0.2, y: 0.4 }, radius: 0.3, direction: -1, orientation: 1 };
    let curves = vec![&curve1, &curve2];
    for &curve in curves {
        ctx.draw_series(
            LineSeries::new(
                curve.get_drawing_data().iter().map(|v| {(v.x, v.y)}),
                yellow.stroke_width(circuit_stroke_size),
                )
            ).unwrap();
    }
    let mut balise1 = Balise {position: Position { x: -0.1, y: 1.8}, part: Part { zone_x: [0.,0.], zone_y: [0.,0.] }, id: 1};
    balise1.generate_zone();
    let mut balise2 = Balise {position: Position { x: 0.3, y: 2.2}, part: Part { zone_x: [0.,0.], zone_y: [0.,0.] }, id: 2};
    balise2.generate_zone();
    let mut balise3 = Balise {position: Position { x: 0.3, y: 0.8}, part: Part { zone_x: [0.,0.], zone_y: [0.,0.] }, id: 3};
    balise3.generate_zone();
    let mut balise4 = Balise {position: Position { x: 0.7, y: 0.4}, part: Part { zone_x: [0.,0.], zone_y: [0.,0.] }, id: 4};
    balise4.generate_zone();

    let mut best_kp: f64;
    
    let iterator: Vec<f64> = linspace(0.015, 0.07, 200).collect();
    let mut hauteurs: Vec<f64> = linspace(0.0, 0.0, 200).collect();
    let mut vars_omega_tot: Vec<f64> = linspace(100000., 100000., 200).collect();
    let mut kps: Vec<f64> = linspace(0.0, 0.0, 200).collect();

    for (i,kp) in iterator.into_iter().enumerate() {
        let resultat_simulation = simulation(&kp, &line1, &line2, &line3, &curve1, &curve2, &balise1, &balise2, &balise3, &balise4);
        match resultat_simulation {
            Some(x) => {
                hauteurs[i] = x.hauteur;
                kps[i] = x.kp_value;
                vars_omega_tot[i] = x.var_omega_tot;
            },
            None => {}
        };
    }

    let max_hauteur = hauteurs.iter().copied().fold(f64::NAN, f64::max);
    let min_var_omega_tot = vars_omega_tot.iter().copied().fold(f64::NAN, f64::min);

    let id_max = hauteurs.iter().position(|a| {
        if *a == max_hauteur {
            true
        } else {
            false
        }
    });
    let id_min = vars_omega_tot.iter().position(|a| {
        if *a == min_var_omega_tot {
            true
        } else {
            false
        }
    });
    match id_max {
        Some(i) => {
            best_kp = kps[i];
            println!("best_kp {}", best_kp);
        },
        None => {
            println!("no match for best kp");
            std::process::exit(1);
        },
    };
    match id_min {
        Some(i) => {
            best_kp = kps[i];
            println!("best_kp for total error {}", best_kp);
        },
        None => {
            println!("no match for best kp");
            std::process::exit(1);
        },
    };

    // au final on prend le critere de minimisation de variation de |omega| totale
    let simulation_finale = simulation(&best_kp, &line1, &line2, &line3, &curve1, &curve2, &balise1, &balise2, &balise3, &balise4);

    let mut centre_capeurs = Vec::new();
    let mut centre_roues = Vec::new();
    let mut cap_lat = Vec::new();
    let mut var_omega_vec = Vec::new();

    match simulation_finale {
        Some(a) => {
            centre_roues = a.centre_roues;
            centre_capeurs = a.centre_capeurs;
            cap_lat = a.cap_lat;
            var_omega_vec = a.var_omega_vec;
        },
        None => {},
    }

    ctx.draw_series(
        LineSeries::new(
            centre_capeurs.iter().map(|v| {(v.x, v.y)}),
            red.stroke_width(2),
            )
        ).unwrap();
    ctx.draw_series(
        LineSeries::new(
            centre_roues.iter().map(|v| {(v.x, v.y)}),
            blue.stroke_width(2),
            )
        ).unwrap();
    // plotter le capteur lateral
    //ctx.draw_series(
    //    LineSeries::new(
    //        cap_lat.iter().map(|v| {(v.x, v.y)}),
    //        green.stroke_width(1),
    //        )
    //    ).unwrap();
    ctx.draw_series(
        LineSeries::new(
            balise1.get_drawing_data().iter().map(|v| {(v.x, v.y)}),
            blue.stroke_width(2),
            )
        ).unwrap();
    ctx.draw_series(
        LineSeries::new(
            balise2.get_drawing_data().iter().map(|v| {(v.x, v.y)}),
            blue.stroke_width(2),
            )
        ).unwrap();
    ctx.draw_series(
        LineSeries::new(
            balise3.get_drawing_data().iter().map(|v| {(v.x, v.y)}),
            blue.stroke_width(2),
            )
        ).unwrap();
    ctx_2.draw_series(
        AreaSeries::new(
            var_omega_vec.iter().map(|v| {(v.x, v.y)}),
            0.0,
            //blue.stroke_width(2),
            &RED.mix(0.4) // Make the series opac
            ).border_style(&green)
        ).unwrap();
}


#[derive(Copy, Clone)]
struct Position {
    x: f64,
    y: f64,
}

struct Balise {
    position: Position,
    part: Part,
    id: u8,
}

impl Balise {
    fn generate_zone(&mut self) {
        self.part.zone_x = [self.position.x - DEMI_L_BALISE, self.position.x + DEMI_L_BALISE];
        self.part.zone_y = [self.position.y - DEMI_H_BALISE, self.position.y + DEMI_H_BALISE];
    }
}

impl GetDrawingData for Balise {
    fn get_drawing_data(self) -> Vec<Position> {
        let mut data = Vec::new();
        data.push(Position {x: self.part.zone_x[0], y: self.position.y});
        data.push(Position {x: self.part.zone_x[1], y: self.position.y});
        data
    }
}

#[derive(Clone)]
struct Vehicule {
    pub omega: f64, // vitesse angulaire
    pub vitesse: f64,
    pub position: Position,
    pub theta: f64, // angle
    pub position_centre_capteurs: Position,
    pub position_capteurs: Vec<Position>,
    pub position_capteur_lat: Position,
}

impl Vehicule {
    fn update_position(&mut self) {
        self.position.x = self.position.x + self.vitesse * DT * self.theta.cos();
        self.position.y = self.position.y + self.vitesse * DT * self.theta.sin();
        self.position_centre_capteurs.x = self.position.x + L_BARRE_ROUES * self.theta.cos();
        self.position_centre_capteurs.y = self.position.y + L_BARRE_ROUES * self.theta.sin();
        self.theta = self.theta + self.omega * DT;
    }
    fn get_position_capteurs(&mut self) {
        let barre = BARRE_8;
        self.position_capteurs = Vec::new();
        for x in barre {
            let pos_x = self.theta.sin() * (x as f64 / 1000.) + self.position_centre_capteurs.x;
            let pos_y = - self.theta.cos() * (x as f64 /1000.) + self.position_centre_capteurs.y;
            self.position_capteurs.push(Position { x: pos_x, y: pos_y })
        }
        let pos_x = self.theta.sin() * POS_REL_CAP_LAT.x + self.position_centre_capteurs.x;
        let pos_y = - self.theta.cos() * POS_REL_CAP_LAT.x + self.position_centre_capteurs.y;
        self.position_capteur_lat = Position {x: pos_x,y: pos_y};
    }
}

#[derive(Copy, Clone)]
struct Moteur {
    pub kp: f64,
    pub default_kp: f64,
    pub kd: f64,
    pub v_l: f64,
    pub v_r: f64,
    pub top_speed_l: f64,
    pub top_speed_r: f64,
}

impl Moteur {
    fn pw_to_tension(&self, pw: f64) -> f64 {
        ( pw / 255. ) * TENSION_MAX
    }
    fn tension_to_speed(&self, tension: f64) -> f64 {
        if tension > 0.35 {
            CARRAC_A * tension + CARRAC_B
        } else {
            0.
        }
    }
    fn update_top_speed(&mut self, id: u8) {
        // possibilite de mettre a jour n'importe quel parametre du module moteur
    }
    fn control(&mut self, error: isize, delta_error: isize) {
        let p = error as f64;
        let pwl: f64;
        let pwr: f64; 
        let pw_corr = self.kp * p + self.kd * delta_error as f64;
        if error > 0 {
            pwl = f64::min(BASE_SPEED_L + pw_corr, self.top_speed_l);
            pwr = f64::max(BASE_SPEED_R - pw_corr, PW_MIN);
        } else if error < 0 {
            pwl = f64::max(BASE_SPEED_L + pw_corr, PW_MIN);
            pwr = f64::min(BASE_SPEED_R - pw_corr, self.top_speed_r);
        } else {
            pwl = self.top_speed_l;
            pwr = self.top_speed_r;
        }
        self.v_l = self.tension_to_speed(self.pw_to_tension(pwl));
        self.v_r = self.tension_to_speed(self.pw_to_tension(pwr));
    }
}

#[derive(Copy, Clone)]
struct Part {
    pub zone_x: [f64;2], // range x 
    pub zone_y: [f64;2], // range y
}

impl Part {
    fn is_in_zone(&self, pos: &Position) -> bool {
        if pos.x > self.zone_x[0] && pos.x < self.zone_x[1] && pos.y > self.zone_y[0] && pos.y < self.zone_y[1] {
            true
        } else {
            false
        }
    }
}

trait CheckContact {
   fn check_contact(self, pos: &Position) -> bool;
}

#[derive(Copy, Clone)]
struct Line {
    part: Part,
    start: Position,
    end: Position,
}

trait GetDrawingData {
    fn get_drawing_data(self) -> Vec<Position>;
}

impl GetDrawingData for Line {
    fn get_drawing_data(self) -> Vec<Position> {
        let mut data = Vec::new();
        data.push(self.start);
        data.push(self.end);
        data
    }
}


impl CheckContact for Line {
    fn check_contact(self, pos: &Position) -> bool {
        if self.part.is_in_zone(pos) {
            if (self.start.x - pos.x).abs() < MARGE {
                true
            } else {
                false
            }
        } else {
            false
        }
    }
}

#[derive(Copy, Clone)]
struct Curve {
    part: Part,
    origine: Position,
    radius: f64,
    direction: i32, // 1 = up, -1 = down
    orientation: i32, // -1 = left, +1 = right
}

impl GetDrawingData for Curve {
    fn get_drawing_data(self) -> Vec<Position> {
        let precision = 200;
        let mut data = Vec::new();
        let alphas;
        if self.direction == 1 {
            alphas = linspace::<f64>(0.,PI,precision);
        } else if self.direction == -1 {
            alphas = linspace::<f64>(PI, PI * 2.,precision);
        } else {
            println!("error due to direction");
            std::process::exit(1);
        }
        for alpha in alphas {
            data.push(Position {x: self.radius * alpha.cos() + self.orientation as f64 * self.radius + self.origine.x,
            y: self.radius * alpha.sin() + self.origine.y });
        }
        data
    }
}

impl CheckContact for Curve {
    fn check_contact(self, pos: &Position) -> bool {
        if self.part.is_in_zone(pos) {
            let x_cap = pos.x - self.origine.x - self.orientation as f64 * self.radius;
            let y_cap = pos.y - self.origine.y;
            let x_opt = (self.radius * x_cap) / (x_cap.powi(2) + y_cap.powi(2)).powf(0.5);
            let y_opt = self.direction as f64 * (self.radius.powi(2) - x_opt.powi(2)).powf(0.5);
            let dist1 = ((x_opt - x_cap).powi(2) + (y_opt - y_cap).powi(2)).powf(0.5);
            let dist2 = ((-x_opt - x_cap).powi(2) + (y_opt - y_cap).powi(2)).powf(0.5);
            let dist = f64::min(dist1, dist2);
            if dist < MARGE {
                true
            } else {
                false
            }
        } else {
            false
        }
    }
}
