use criterion::{black_box, criterion_group, criterion_main, Criterion};
use fvd_rs::track::Track;

fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("get_spline", |b| {
        let track =
            black_box(serde_json::from_str::<Track>(include_str!("../week_11.json")).unwrap());

        b.iter(|| serde_json::to_string(&track.get_spline()))
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
