use std::collections::VecDeque;
use std::ops::*;
use std::time::{Instant, Duration};


/// moving (rolling) average
#[derive(Debug, Clone)]
pub struct RollingAverage<T> {
    hist: VecDeque<T>,
    sum: T,
    size: usize,
}

impl<T> RollingAverage<T>
// moments like this I miss dating duck typing
where T: AddAssign + SubAssign + Div + Add
    + std::convert::From<<T as std::ops::Div>::Output> + Copy + Default 
    + std::ops::Div<Output = T> + From<u32> + std::fmt::Debug + std::fmt::Display
{
    pub fn new(size: usize) -> Self {
        let this = RollingAverage {
            hist: VecDeque::with_capacity(size),
            sum: (0 as u32).into(),
            size,
        };
        // info!("xxx {:?}", this);
        this
    }

    pub fn add(&mut self, val: T) where <T as std::ops::Add>::Output: std::fmt::Display {
        self.hist.push_back(val);
        // info!("add {} + {} = {}", self.sum, val, self.sum + val);
        self.sum += val;
        if self.hist.len() > self.size {
            self.sum -= self.hist.pop_front().unwrap();
        }
    }

    pub fn get(&self) -> T {
        if self.hist.len() == 0 {
            return 0.into()
        }
        self.sum / (self.hist.len() as u32).into()
    }
}

// pub struct AvgSpeed<T> {
//     avg: RollingAverage<T>,
//     prev: T,
//     last_chunk: Instant,
//     // prev: T,
// }

// impl<T: From<u64> + AddAssign + Sub + SubAssign + Div + Copy + From<<T as std::ops::Div>::Output>> AvgSpeed<T> {
//     pub fn new() -> Self {
//         AvgSpeed {
//             avg: RollingAverage::new(100),
//             prev: (0 as u64).into(),
//             last_chunk: Instant::now(),
//         }
//     }
//     pub fn add(&mut self, val: T) {
//         let db = val - self.prev;
//         self.avg.add(get_speed(db, &Instant::now().duration_since(self.last_chunk)));
//         self.last_chunk = Instant::now();
//         self.prev_bytes = total_bytes;
//     }
//     pub fn get(&self) -> u64 {
//         self.avg.get()
//     }
// }

// pub fn get_speed(x: u64, ela: &Duration) -> u64 {
//     if *ela >= Duration::from_nanos(1) && x < std::u64::MAX/1_000_000_000 {
//         x * 1_000_000_000 / ela.as_nanos() as u64
//     }
//     else if *ela >= Duration::from_micros(1) && x < std::u64::MAX/1_000_000 {
//         x * 1_000_000 / ela.as_micros() as u64
//     }
//     else if *ela >= Duration::from_millis(1) && x < std::u64::MAX/1_000 {
//         x * 1_000 / ela.as_millis() as u64
//     }
//     else if *ela >= Duration::from_secs(1) {
//         x / ela.as_secs() as u64
//     }
//     else {
//         // what the hell are you?
//         std::u64::MAX
//     }
// }
