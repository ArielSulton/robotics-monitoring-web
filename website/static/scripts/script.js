const rollSpan = document.getElementById('roll');
const pitchSpan = document.getElementById('pitch');
const yawSpan = document.getElementById('yaw');

async function fetchImu() {
    try {
        const res = await fetch('/orientation');
        const data = await res.json();
        if (data) {
            rollSpan.textContent = data.roll.toFixed(2);
            pitchSpan.textContent = data.pitch.toFixed(2);
            yawSpan.textContent = data.yaw.toFixed(2);
        }
    } catch (e) {
        console.error("Failed to fetch imu", e);
    }
}

setInterval(fetchImu, 200);