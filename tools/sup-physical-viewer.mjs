import {BuildingViewer} from '/sup/viewer.js';
import {BufferGeometry, EdgesGeometry, Float32BufferAttribute, LineBasicMaterial, LineSegments, ShapeUtils, Vector2, Vector3} from '/sup/vendor/three.module.js';

// foundation здесь только штатный транспорт произвольной сетки СУП.
// Предметный тип и источник изделия сохраняются отдельно, семантика FB не меняется.
export function physicalScene(source) {
    if (source.format !== 'physical_body_v1' || !Array.isArray(source.blocks))
        throw new Error('Ожидался physical_body_v1');
    let bodyCount = 0, triangleCount = 0;
    const elements = source.blocks.map(block => {
        const world = [], triangles = [];
        for (const body of block.bodies) {
            const offset = world.length;
            if (!body.vertices?.length || !Array.isArray(body.faces)) throw new Error(`Нет тела ${block.id}`);
            for (const point of body.vertices) {
                if (!Array.isArray(point) || point.length !== 3 || !point.every(Number.isFinite))
                    throw new Error(`Неверная вершина ${block.id}`);
                world.push(point);
            }
            for (const face of body.faces) {
                if (face.length < 3 || face.some(i => !Number.isInteger(i) || i < 0 || i >= body.vertices.length))
                    throw new Error(`Неверная грань ${block.id}`);
                const points = face.map(i => new Vector3(...body.vertices[i]));
                const normal = new Vector3();
                for (let i = 0; i < points.length; i++) {
                    const a = points[i], b = points[(i + 1) % points.length];
                    normal.x += (a.y - b.y) * (a.z + b.z);
                    normal.y += (a.z - b.z) * (a.x + b.x);
                    normal.z += (a.x - b.x) * (a.y + b.y);
                }
                if (normal.lengthSq() < 1e-16) throw new Error(`Вырожденная грань ${block.id}`);
                const drop = Math.abs(normal.x) >= Math.abs(normal.y) && Math.abs(normal.x) >= Math.abs(normal.z) ? 0
                    : Math.abs(normal.y) >= Math.abs(normal.z) ? 1 : 2;
                const axes = [0, 1, 2].filter(axis => axis !== drop);
                const contour = face.map(i => new Vector2(body.vertices[i][axes[0]], body.vertices[i][axes[1]]));
                for (const triangle of ShapeUtils.triangulateShape(contour, [])) {
                    const indices = triangle.map(i => face[i]);
                    const [a, b, c] = indices.map(i => new Vector3(...body.vertices[i]));
                    if (b.sub(a).cross(c.sub(a)).dot(normal) < 0) [indices[1], indices[2]] = [indices[2], indices[1]];
                    triangles.push(indices.map(i => offset + i));
                }
            }
            bodyCount++;
        }
        const min = [0, 1, 2].map(axis => Math.min(...world.map(point => point[axis])));
        const max = [0, 1, 2].map(axis => Math.max(...world.map(point => point[axis])));
        const center = min.map((value, axis) => (value + max[axis]) / 2);
        triangleCount += triangles.length;
        return {id: block.id, kind: 'foundation', center, size: max.map((value, axis) => value - min[axis]),
            quaternion: [0, 0, 0, 1], vertices: world.map(point => point.map((value, axis) => value - center[axis])), triangles,
            color: block.kind === 'ordinary' ? 0xd3af72 : block.kind === 'node_T' ? 0x779ba3 : 0xa990ad,
            properties: {fbKind: block.kind, courseIndex: block.course_index, sourceIds: block.source_ids.join(', '), cut: block.cut}};
    });
    return {version: 1, units: 'mm', elements, bodyCount, triangleCount};
}

const status = document.querySelector('#status'), course = document.querySelector('#course');
try {
    status.textContent = 'Загрузка physical_body_v1…';
    const response = await fetch('/target/banya-work/walls-physical.json', {cache: 'no-store'});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const source = await response.json();
    const warningGroups = new Map();
    for (const warning of source.warnings ?? []) {
        const group = warningGroups.get(warning.code);
        if (group) group.count++;
        else warningGroups.set(warning.code, {message: warning.message, count: 1});
    }
    if (warningGroups.size) {
        document.querySelector('#warnings').hidden = false;
        document.querySelector('#warning-summary').textContent = `Предупреждения раскладки: ${source.warnings.length}`;
        for (const [code, warning] of warningGroups) {
            const paragraph = document.createElement('p');
            paragraph.textContent = `${code} (${warning.count}): ${warning.message}`;
            document.querySelector('#warning-list').append(paragraph);
        }
    }
    status.textContent = 'Подготовка физических граней…';
    const scene = physicalScene(source);
    const viewer = new BuildingViewer(document.querySelector('#viewport'), {
        onError: error => { status.textContent = String(error); },
        onSelect: element => { document.querySelector('#details').textContent = element
            ? `${element.id}\nТип: ${element.properties.fbKind}\nВенец: ${Number(element.properties.courseIndex) + 1}\n${element.properties.sourceIds}` : ''; }
    });
    let outlines;
    function loadScene(elements) {
        if (outlines) { viewer.scene.remove(outlines); outlines.geometry.dispose(); outlines.material.dispose(); }
        viewer.load({...scene, elements});
        const positions = [];
        for (const element of viewer.data.elements) {
            const mesh = viewer.instances.get(element.id).mesh;
            const edges = new EdgesGeometry(mesh.geometry, 20), attribute = edges.getAttribute('position');
            for (let i = 0; i < attribute.count; i++) positions.push(
                attribute.getX(i) * element.size.x + element.center.x,
                attribute.getY(i) * element.size.y + element.center.y,
                attribute.getZ(i) * element.size.z + element.center.z);
            edges.dispose();
        }
        const geometry = new BufferGeometry();
        geometry.setAttribute('position', new Float32BufferAttribute(positions, 3));
        outlines = new LineSegments(geometry, new LineBasicMaterial({color: 0x574c40}));
        viewer.scene.add(outlines);
        viewer.invalidate();
    }
    loadScene(scene.elements);
    const courses = [...new Set(scene.elements.map(element => element.properties.courseIndex))].sort((a, b) => a - b);
    for (const index of courses) course.add(new Option(String(index + 1), String(index)));
    status.textContent = `${source.project_name || 'Баня'} · ${scene.elements.length} изделий · ${scene.bodyCount} тел · ${scene.triangleCount} треугольников`;
    course.addEventListener('change', () => {
        loadScene(course.value === 'all' ? scene.elements
            : scene.elements.filter(element => element.properties.courseIndex === Number(course.value)));
    });
    document.querySelector('#fit').addEventListener('click', () => viewer.fit());
    document.querySelector('#save-png').addEventListener('click', () => {
        viewer.renderScene();
        const link = document.createElement('a');
        link.download = 'banya-sup-physical.png';
        link.href = viewer.canvas.toDataURL('image/png');
        link.click();
    });
    addEventListener('pagehide', () => { outlines?.geometry.dispose(); outlines?.material.dispose(); viewer.dispose(); }, {once: true});
} catch (error) {
    status.textContent = `Ошибка просмотра: ${error.message}`;
}
