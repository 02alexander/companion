use common::EKF;
use rerun::dataframe::{QueryEngine, QueryExpression, SparseFillStrategy};
use rerun::external::arrow;
use rerun::external::arrow::array::{Array, Float64Array, ListArray, PrimitiveArray};
use rerun::external::arrow::datatypes::DurationNanosecondType;
use rerun::{ChunkStoreConfig, EntityPath};

fn main() -> anyhow::Result<()> {
    let engines = QueryEngine::from_rrd_filepath(&ChunkStoreConfig::DEFAULT, "changing_sign.rrd")?;

    let (store_id, engine) = engines.iter().next().unwrap();

    let query = QueryExpression {
        filtered_index: Some("sample_time".into()),
        view_contents: Some(
            [
                (EntityPath::try_from("/ticks").unwrap(), None),
                (EntityPath::try_from("/control").unwrap(), None),
            ]
            .into_iter()
            .collect(),
        ),
        sparse_fill_strategy: SparseFillStrategy::LatestAtGlobal,
        ..Default::default()
    };

    let query_handle = engine.query(query.clone());
    let record_batches: Vec<_> = query_handle.batch_iter().collect();
    println!("{:#?}", query_handle.schema());

    let batch = arrow::compute::concat_batches(query_handle.schema(), &record_batches)?;

    let control: &[f64] = batch
        .column_by_name("/control:Scalar")
        .unwrap()
        .as_any()
        .downcast_ref::<ListArray>()
        .unwrap()
        .values()
        .as_any()
        .downcast_ref::<Float64Array>()
        .unwrap()
        .values();

    let ticks: &[f64] = batch
        .column_by_name("/ticks:Scalar")
        .unwrap()
        .as_any()
        .downcast_ref::<ListArray>()
        .unwrap()
        .values()
        .as_any()
        .downcast_ref::<Float64Array>()
        .unwrap()
        .values();

    let sample_times: &[i64] = batch
        .column_by_name("sample_time")
        .unwrap()
        .as_any()
        .downcast_ref::<PrimitiveArray<DurationNanosecondType>>()
        .unwrap()
        .values();

    let n = sample_times.len() - 1;
    let diffs = sample_times.iter().skip(1).zip(sample_times.iter()).map(|(a,b)| *a - *b);
    let ts = diffs.sum::<i64>() as f64 / n as f64 * 1e-9;
    println!("ts={ts}");

    // let filter = EKF::new();

    println!("{:?}", store_id);

    Ok(())
}
