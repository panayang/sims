// src/main.rs (DEBUGGING VERSION)

use plotters::prelude::*;
use std::f64;

// --- Constants ---
const R: f64 = 0.13; // Changed R to 0.13 as per your output
const H: f64 = 1e-6;
const FILE_NAME: &str = "y_and_y_prime_curves.png";
const DEBUG_X: f64 = 0.195; // The point returning NaN

// The original function y(x) with ABS-VALUE FIX and DEBUGGING PRINTS
fn y(x: f64) -> f64 {
    let r = R;
    let under_root = x * (2.0 * r - x);

    // Domain Check 1: Square root argument
    if under_root < 0.0 {
        eprintln!("[y(x) FAIL] x={:.6}: under_root < 0", x);
        return f64::NAN;
    }

    let sqrt_term = under_root.sqrt();

    // FIX: Use abs value for the denominator (r-x) to ensure ln argument is positive
    let den_abs = (r - x).abs();

    // Domain Check 2: Denominator
    if den_abs < 1e-9 {
        eprintln!("[y(x) FAIL] x={:.6}: Denominator |r-x| is zero", x);
        return f64::NAN;
    }

    let t1 = sqrt_term;
    let t2_ln_arg = (r - sqrt_term) / den_abs;

    // Domain Check 3: Logarithm argument
    if t2_ln_arg <= 0.0 {
        if (x - DEBUG_X).abs() < 5e-6 {
            // Print only near the failing point
            eprintln!(
                "[y(x) FAIL] x={:.6}: ln argument is negative. Arg={:.4e}",
                x, t2_ln_arg
            );
        }
        return f64::NAN;
    }

    let result = t1 + r * t2_ln_arg.ln();

    if result.is_nan() || result.is_infinite() {
        eprintln!(
            "[y(x) FAIL] x={:.6}: Result is NaN/Inf. ln Arg={:.4e}",
            x, t2_ln_arg
        );
    }

    result
}

// Numerical derivative y'(x)_num with DEBUGGING PRINTS
fn y_prime_num(x: f64) -> f64 {
    let y_plus_h = y(x + H);
    let y_minus_h = y(x - H);

    if x == DEBUG_X {
        eprintln!("--- DEBUGGING y'({:.3}) ---", x);
        eprintln!("y(x+H) = y({:.6}) = {:.6e}", x + H, y_plus_h);
        eprintln!("y(x-H) = y({:.6}) = {:.6e}", x - H, y_minus_h);
    }

    if y_plus_h.is_nan() || y_minus_h.is_nan() {
        eprintln!("[y'(x) FAIL] x={:.6}: One of the y(x+/-H) calls failed.", x);
        return f64::NAN;
    }

    (y_plus_h - y_minus_h) / (2.0 * H)
}

// ... (main function content below) ...

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // ... (Constants are defined at the top) ...
    let target_value = 1.7320508075688772;
    let solution_x = 3.0 * R / 2.0;
    let prime_at_solution = y_prime_num(solution_x);

    // ... (All numerical check printing remains the same) ...

    println!("--- Numerical Results (R = {}) ---", R);
    println!("Theoretical solution x for y'(x) = sqrt(3): {}", solution_x);
    println!("Target value (sqrt(3)): {:.6}", target_value);
    println!("Numerical y'({}) : {:.6}", solution_x, prime_at_solution);
    println!(
        "Difference: {:.2e}",
        (prime_at_solution - target_value).abs()
    );

    if (prime_at_solution - target_value).abs() < 1e-4 {
        println!("\n✅ The numerical result is very close to the theoretical target!");
    } else {
        println!("\n❌ The numerical result is outside expected range.");
    }
    println!("--------------------------------------\n");

    // ... (The rest of the plotting code remains the same, using the solid line fix) ...

    // --- 2. Plotting the Curves ---
    let root = BitMapBackend::new(FILE_NAME, (10240, 7680)).into_drawing_area();
    root.fill(&WHITE)?;

    let x_min = 0.0 + 1e-3;
    let x_max = 2.0 * R - 1e-3;
    const SINGULARITY: f64 = R;

    // --- Determine Y range ---
    let num_points = 500;
    let step = (x_max - x_min) / (num_points as f64);
    let mut y_values = Vec::new();

    for i in 0..num_points {
        let x = x_min + step * (i as f64);

        // Skip the immediate singular region for plotting robustness
        if (x - SINGULARITY).abs() < 1e-5 {
            continue;
        }

        let y_val = y(x);
        let yp_val = y_prime_num(x);

        // Only collect finite values for auto-scaling
        if y_val.is_finite() {
            y_values.push(y_val);
        }
        if yp_val.is_finite() {
            y_values.push(yp_val);
        }
    }

    let y_min = y_values.iter().cloned().fold(f64::INFINITY, f64::min);
    let y_max = y_values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let margin = 0.1 * (y_max - y_min).max(1.0);
    let plot_y_min = y_min - margin;
    let plot_y_max = y_max + margin;

    let mut chart = ChartBuilder::on(&root)
        .caption(
            format!("y(x) and y'(x) (R={})", R),
            ("sans-serif", 40).into_font(),
        )
        .margin(10)
        .x_label_area_size(30)
        .y_label_area_size(50)
        // Adjust the Y-range to center around the expected target (1.732) if the calculated range is massive
        .build_cartesian_2d(
            x_min..x_max,
            (-5.0_f64).min(plot_y_min)..5.0_f64.max(plot_y_max),
        )?; // Use a clamped Y range

    chart.configure_mesh().draw()?;

    // Plot y(x)
    chart
        .draw_series(LineSeries::new(
            (0..num_points).filter_map(|i| {
                let x = x_min + step * (i as f64);
                if (x - SINGULARITY).abs() < 1e-5 {
                    return None;
                }
                let y_val = y(x);
                if y_val.is_finite() {
                    Some((x, y_val))
                } else {
                    None
                }
            }),
            &BLUE.mix(0.8),
        ))?
        .label("y(x) (Original Function)")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLUE));

    // Plot y'(x)_num
    chart
        .draw_series(LineSeries::new(
            (0..num_points).filter_map(|i| {
                let x = x_min + step * (i as f64);
                if (x - SINGULARITY).abs() < 1e-5 {
                    return None;
                }
                let yp_val = y_prime_num(x);
                if yp_val.is_finite() {
                    Some((x, yp_val))
                } else {
                    None
                }
            }),
            &RED.mix(0.8),
        ))?
        .label("y'(x) (Numerical Derivative)")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));

    // Draw solid target line
    chart
        .draw_series(LineSeries::new(
            vec![(x_min, target_value), (x_max, target_value)],
            GREEN.mix(0.6).stroke_width(2),
        ))?
        .label("Target y' = sqrt(3)")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &GREEN));

    // Mark the solution point x = 0.195
    chart.draw_series(PointSeries::of_element(
        vec![(solution_x, target_value)],
        5,
        &BLACK,
        &|c, s, st| {
            EmptyElement::at(c)
                + Circle::new((0, 0), s, st.filled())
                + Text::new(
                    format!("x={:.2}", solution_x),
                    (10, -10),
                    ("sans-serif", 15).into_font(),
                )
        },
    ))?;

    // Draw the legend
    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()?;

    println!("\nPlot saved to {}", FILE_NAME);

    Ok(())
}
