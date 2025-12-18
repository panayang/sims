use ode_solvers::dopri5::*;
use ode_solvers::*;
use plotters::prelude::*;
use std::f64;

// --- Physical and Numerical Constants (Constrained) ---
const A: f64 = 1.5e-6;
const Y_INIT: f64 = 1.7;

// Constraint: Max X is physically limited to < 10,000.1
const MAX_X_BOUND: f64 = 10_000.1;
const SLOPE_TOLERANCE: f64 = 1.0e-12; // Precision for the optimal slope

// Initial bracket targeting the X=10,000 solution (s_avg = -0.00017)
const S_LOW_INIT: f64 = -0.0; // Shallow slope (Guessed to FAIL: hits bound, X > 10k)
const S_HIGH_INIT: f64 = -0.050; // Steep slope (Guessed to SUCCEED: X < 10k)

// --- ODE Definition ---
type State = Vector2<f64>;
type Time = f64;

struct DifferentialEquation;

impl System<Time, State> for DifferentialEquation {
    fn system(&self, _t: Time, y: &State, dy: &mut State) {
        // y[0] = y, y[1] = y'
        dy[0] = y[1];

        // y'' = A * (y'^2 + 1) / (1 + A*y)
        let numerator = A * (y[1].powi(2) + 1.0);
        let denominator = 1.0 + A * y[0];

        if denominator.abs() < 1.0e-12 {
            dy[1] = 0.0;
        } else {
            dy[1] = numerator / denominator;
        }
    }

    // CRITICAL FIX: Stop integration when y <= 0 AND the curve is descending (y' <= 0).
    // This ignores the spurious initial dips and the irrelevant second crossing for large X.
    fn solout(&mut self, _x: Time, y: &State, _dy: &State) -> bool {
        y[0] <= 0.000001 && y[1] <= 0.000001
    }
}

// --- Integration Wrapper (Finds X-Intercept) ---
// Returns the final X value where y=0, or None if the boundary was hit.
fn x_at_y_zero(s_init: f64) -> Option<(f64, Vec<Time>, Vec<State>)> {
    if s_init >= 0.0 {
        return None;
    }

    let y0 = State::new(Y_INIT, s_init);
    let start_x = 0.0;
    let step_size = 0.1;

    let system = DifferentialEquation;
    let mut stepper = Dopri5::new(
        system,
        start_x,
        MAX_X_BOUND,
        step_size,
        y0,
        1.0e-12,
        1.0e-12,
    );

    match stepper.integrate() {
        Ok(_) => {
            let x_data = stepper.x_out();

            // Success if the last X is LESS than the MAX_X_BOUND
            if *x_data.last().unwrap_or(&MAX_X_BOUND) < MAX_X_BOUND {
                Some((
                    *x_data.last().unwrap(),
                    x_data.clone(),
                    stepper.y_out().clone(),
                ))
            } else {
                // Did not cross y=0 within the domain (FAILURE for bisection)
                None
            }
        }
        Err(_) => None,
    }
}

// --- CORE OPTIMIZATION FUNCTION (Bisection Search for MAX X) ---
fn find_optimal_slope_bisection() -> Option<(f64, f64, Vec<Time>, Vec<State>)> {
    // 1. Establish the Bracketing Range for the Slope
    let mut s_low = S_LOW_INIT; // Shallow slope (should FAIL: hits bound)
    let mut s_high = S_HIGH_INIT; // Steep slope (should SUCCEED: crosses y=0)

    // Test s_high to get initial best X (X_high)
    let mut current_best = x_at_y_zero(s_high);
    let (mut x_high, mut x_data_best, mut y_data_best) = match current_best {
        Some((x, xd, yd)) => (x, xd, yd),
        None => {
            eprintln!(
                "Error: Initial s_high ({:.6}) failed to cross y=0. Try a steeper slope.",
                s_high
            );
            return None;
        }
    };

    // Test s_low to ensure it fails (hits MAX_X_BOUND).
    if x_at_y_zero(s_low).is_some() {
        // If s_low succeeds, it means the true maximum X is much smaller than 10k.
        eprintln!(
            "Error: s_low ({:.6}) is too steep (it crosses y=0). Maximum X is likely much smaller than 10k.",
            s_low
        );
        s_low /= 10.0; // Try a much shallower slope
        if x_at_y_zero(s_low).is_some() {
            eprintln!("Fatal Error: Cannot bracket the problem for X < 10,000.");
            return None;
        }
    }

    println!(
        "Initial Bracket: s_low={:.15} (Fails to cross), s_high={:.15} (X={:.4})",
        s_low, s_high, x_high
    );

    // 2. Binary Search Loop (Bisection)
    for i in 0..100 {
        let s_mid = (s_low + s_high) / 2.0;

        if (s_high - s_low).abs() < SLOPE_TOLERANCE {
            break;
        }

        let mid_result = x_at_y_zero(s_mid);

        match mid_result {
            Some((x_mid, x_data_mid, y_data_mid)) => {
                // Success: s_mid crossed y=0. This is a potential new maximum X.
                s_high = s_mid; // Move s_high towards the shallow (X-maximizing) side

                if x_mid > x_high {
                    x_high = x_mid;
                    x_data_best = x_data_mid;
                    y_data_best = y_data_mid;
                }
            }
            None => {
                // Failure: Slope was too shallow (hit MAX_X_BOUND).
                s_low = s_mid; // Move s_low away from s_high (towards the zero slope)
            }
        }
        println!(
            "Iteration {}: s_low={:.15}, s_high={:.15}",
            i + 1,
            s_low,
            s_high
        );
    }

    // The final result is the highest X found (x_high) and the corresponding shallow slope (s_high).
    Some((x_high, s_high, x_data_best, y_data_best))
}

// --- Plotting Function (Adjusted for X-Intercept) ---
fn plot_curve(
    x_data: &[Time],
    y_data: &[State],
    max_x: f64,
    s_opt: f64,
    filename: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new(filename, (12000, 8000)).into_drawing_area();
    root.fill(&WHITE)?;

    // Determine the Y range based on data, constrained to physical relevance (avoiding huge Y values)
    let min_y = y_data
        .iter()
        .map(|v| v[0])
        .fold(f64::INFINITY, f64::min)
        .min(-0.1);
    let max_y = y_data
        .iter()
        .map(|v| v[0])
        .fold(f64::NEG_INFINITY, f64::max)
        .max(Y_INIT * 1.05);

    let mut chart = ChartBuilder::on(&root)
        .caption(
            format!(
                "Optimal Constrained Curve (s={:.15}) to y=0 at x={:.10}",
                s_opt, max_x
            ),
            ("sans-serif", 30).into_font(),
        )
        .margin(10)
        .x_label_area_size(40)
        .y_label_area_size(40)
        .build_cartesian_2d(0.0..max_x * 1.05, min_y..max_y)?;

    chart.configure_mesh().draw()?;

    chart
        .draw_series(LineSeries::new(
            x_data.iter().zip(y_data.iter()).map(|(x, y)| (*x, y[0])),
            &RED,
        ))?
        .label("y(x)")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));

    chart.draw_series(PointSeries::of_element(
        vec![(
            *x_data.last().unwrap_or(&0.0),
            y_data.last().unwrap_or(&State::new(0.0, 0.0))[0],
        )],
        5,
        ShapeStyle::from(&BLACK).filled(),
        &|coord, size, style| Circle::new(coord, size, style),
    ))?;

    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()?;

    Ok(())
}

// --- New Function: Finds the minimum y-value (y_min) and its location (x_min) ---
fn get_min_y(s_init: f64) -> Option<(f64, f64)> {
    // We use a large bound here to ensure the curve has space to reach its vertex
    const MIN_Y_BOUND: f64 = 5_000_000.0;
    const X_START_OFFSET: f64 = 1.0e-10; // New starting X to bypass X=0 checks

    // Extrapolate the state (y, y') for the new starting X_START_OFFSET.
    // Since the step is tiny, y' is constant (s_init), and y is linear:
    let y_start = Y_INIT + s_init * X_START_OFFSET;
    let y0_offset = State::new(y_start, s_init);

    // Define the VertexFinder struct to carry the ODE context
    struct VertexFinder {
        ode_system: DifferentialEquation,
    }

    impl System<Time, State> for VertexFinder {
        fn system(&self, t: Time, y: &State, dy: &mut State) {
            self.ode_system.system(t, y, dy);
        }

        // Vertex stopping condition (y'=0)
        fn solout(&mut self, _x: Time, y: &State, _dy: &State) -> bool {
            y[1].abs() < 1.0e-12
        }
    }

    let vertex_finder = VertexFinder {
        ode_system: DifferentialEquation,
    };

    let mut stepper = Dopri5::new(
        vertex_finder,
        X_START_OFFSET, // Start integration at the offset
        MIN_Y_BOUND,
        0.1,
        y0_offset, // Start with the offset state
        1.0e-12,
        1.0e-12,
    );

    // Integrate directly without the problematic manual step() call
    match stepper.integrate() {
        Ok(_) => {
            let x_data = stepper.x_out();
            let y_data = stepper.y_out();

            // Check if termination happened (y' crossed 0)
            if x_data.len() > 1 && y_data.last().map(|y| y[1].abs() < 1.0e-10).unwrap_or(false) {
                let x_min = *x_data.last().unwrap();
                let y_min = y_data.last().unwrap()[0];
                Some((y_min, x_min))
            } else {
                // Vertex not reached within the domain or integration failed
                None
            }
        }
        Err(_) => None,
    }
}

// --- Main Function ---
fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Inside the main function's match statement:
    match find_optimal_slope_bisection() {
        Some((max_x_val, optimal_s, x_data, y_data)) => {
            // --- Find Minimum from successful data array ---
            let mut min_y_val = f64::INFINITY;
            let mut x_at_min = 0.0;

            // Iterate through all data points saved during the integration
            for (i, state) in y_data.iter().enumerate() {
                let current_y = state[0];
                if current_y < min_y_val {
                    min_y_val = current_y;
                    x_at_min = x_data[i];
                }
            }
            // --- End Find Minimum ---

            println!("\n✅ Optimization Successful (Bisection Converged):");
            println!("   Optimal Initial Slope (y'(0)) = {:.15}", optimal_s);
            println!("   The Largest X where y=0 is: {:.10}", max_x_val);

            // Use the calculated minimum value from the data array
            println!("   Minimum Y-value (Vertex) is: {:.15}", min_y_val);
            println!("   X position of Minimum Y (X_vertex) is: {:.10}", x_at_min);

            println!(
                "   Final Y at X-intercept is: {:.15}",
                y_data.last().unwrap()[0]
            );

            // ... (rest of the plotting and final printout)

            let filename = "solution_curve_constrained_x.png";
            plot_curve(&x_data, &y_data, max_x_val, optimal_s, filename)?;
            println!("   Plot saved to '{}'", filename);

            // NEW CODE: Calculate and print the minimum value and its location
            if let Some((min_y_val, x_at_min)) = get_min_y(optimal_s) {
                println!("   Minimum Y-value (Vertex) is: {:.15}", min_y_val);
                println!("   X position of Minimum Y (X_vertex) is: {:.10}", x_at_min);
            } else {
                println!(
                    "   Warning: Could not accurately calculate the minimum Y-value (Vertex)."
                );
            }
        }
        None => {
            println!("\n❌ Optimization failed. Cannot find a suitable initial slope bracket.");
        }
    }

    Ok(())
}
