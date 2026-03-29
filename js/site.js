// =============================================
// GutAngle Website - site.js
// Controls: Login modal, tab switching, APK download
// =============================================

let downloadMode = false;

function openLogin(mode) {
    downloadMode = (mode === 'download');
    document.getElementById('login-modal').style.display = 'block';
    document.body.style.overflow = 'hidden';
    
    // Update button labels dynamically
    if (!downloadMode) {
        document.getElementById('signin-submit-btn').textContent = 'Sign in to Dashboard';
        document.querySelector('#signupForm .primary-btn').textContent = 'Create Account';
    } else {
        document.getElementById('authTitle').textContent = 'Download GutAngle App';
        document.getElementById('authCaption').innerHTML = '<span>Please provide your details to download the mobile app.</span>';
        document.querySelector('#signupForm .primary-btn').textContent = 'Submit & Download APK';
        document.getElementById('signin-submit-btn').textContent = 'Sign in & Download APK';
    }
    
    initLightRays();
}

function closeLogin() {
    document.getElementById('login-modal').style.display = 'none';
    document.body.style.overflow = '';
}

function switchTab(tab) {
    const pill = document.getElementById('tabPill');
    const signupTab = document.getElementById('tabSignup');
    const signinTab = document.getElementById('tabSignin');
    const signupForm = document.getElementById('signupForm');
    const signinForm = document.getElementById('signinForm');
    const title = document.getElementById('authTitle');
    const caption = document.getElementById('authCaption');

    if (tab === 'signup') {
        pill.dataset.mode = 'signup';
        signupTab.dataset.active = 'true';
        signinTab.dataset.active = 'false';
        signupForm.dataset.active = 'true';
        signinForm.dataset.active = 'false';
        title.textContent = 'Create GUTANGLE workspace';
        caption.innerHTML = '<span>Start by creating an account to get the app.</span><span style="display:block;font-weight:bold;color:#fff;">Sign up is free.</span>';
    } else {
        pill.dataset.mode = 'signin';
        signinTab.dataset.active = 'true';
        signupTab.dataset.active = 'false';
        signinForm.dataset.active = 'true';
        signupForm.dataset.active = 'false';
        title.textContent = 'Welcome Back';
        caption.innerHTML = '<span>Enter your credentials to access the dashboard.</span>';
    }
}

function handleSignup(e) {
    e.preventDefault();
    const name = document.getElementById('su-name').value.trim();
    const email = document.getElementById('su-email').value.trim();
    const phone = document.getElementById('su-phone').value.trim();
    const age = document.getElementById('su-age').value.trim();
    const address = document.getElementById('su-address').value.trim();
    const pass = document.getElementById('su-pass').value;
    const err = document.getElementById('su-error');

    if (!name || !email || !phone || !age || !address || !pass) {
        err.textContent = 'Please fill in all fields.';
        err.style.display = 'block';
        return;
    }
    if (pass.length < 6) {
        err.textContent = 'Password must be at least 6 characters.';
        err.style.display = 'block';
        return;
    }
    err.style.display = 'none';

    // Store locally (IndexedDB or localStorage)
    const users = JSON.parse(localStorage.getItem('gutangle_users') || '[]');
    if (users.find(u => u.email === email)) {
        err.textContent = 'An account with this email already exists. Please sign in.';
        err.style.display = 'block';
        return;
    }
    users.push({ name, email, phone, age, address, pass });
    localStorage.setItem('gutangle_users', JSON.stringify(users));
    localStorage.setItem('gutangle_current_user', JSON.stringify({ name, email }));

    closeLogin();
    triggerDownload();
}

function handleSignin(e) {
    e.preventDefault();
    const email = document.getElementById('si-email').value.trim();
    const pass = document.getElementById('si-pass').value;
    const err = document.getElementById('si-error');

    const users = JSON.parse(localStorage.getItem('gutangle_users') || '[]');
    const user = users.find(u => u.email === email && u.pass === pass);

    if (!user) {
        err.textContent = 'Invalid email or password.';
        err.style.display = 'block';
        return;
    }
    err.style.display = 'none';
    localStorage.setItem('gutangle_current_user', JSON.stringify({ name: user.name, email }));

    closeLogin();
    triggerDownload();
}

function triggerDownload() {
    const toast = document.getElementById('download-toast');
    toast.style.display = 'block';

    // Redirect to Google Drive folder
    const driveLink = 'https://drive.google.com/drive/folders/1sCZWGTOVisNZXRqbuj7JPTC5s8uiKrIi?usp=sharing';
    window.open(driveLink, '_blank');

    setTimeout(() => {
        toast.style.opacity = '0';
        toast.style.transition = 'opacity 1s';
        setTimeout(() => { toast.style.display = 'none'; toast.style.opacity = '1'; }, 1000);
    }, 5000);
}

// ---- Light Rays Canvas Effect (same as login page) ----
function initLightRays() {
    const canvas = document.getElementById('lightRaysCanvas');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');

    function resize() {
        canvas.width = window.innerWidth;
        canvas.height = window.innerHeight;
    }
    resize();
    window.addEventListener('resize', resize);

    const rays = Array.from({ length: 12 }, () => ({
        x: Math.random() * window.innerWidth,
        angle: (Math.random() - 0.5) * 0.8,
        width: 40 + Math.random() * 80,
        opacity: 0.02 + Math.random() * 0.04,
        speed: (Math.random() - 0.5) * 0.3
    }));

    function draw() {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        rays.forEach(ray => {
            ray.x += ray.speed;
            if (ray.x < -200) ray.x = canvas.width + 200;
            if (ray.x > canvas.width + 200) ray.x = -200;

            ctx.save();
            ctx.translate(ray.x, 0);
            ctx.rotate(ray.angle);
            const grad = ctx.createLinearGradient(0, 0, 0, canvas.height);
            grad.addColorStop(0, `rgba(220,38,38,${ray.opacity})`);
            grad.addColorStop(1, 'transparent');
            ctx.fillStyle = grad;
            ctx.fillRect(-ray.width / 2, 0, ray.width, canvas.height * 1.5);
            ctx.restore();
        });
        requestAnimationFrame(draw);
    }
    draw();
}

// Auto-start if returning user
document.addEventListener('DOMContentLoaded', () => {
    const user = localStorage.getItem('gutangle_current_user');
    if (user) {
        const u = JSON.parse(user);
        console.log(`Welcome back, ${u.name}`);
    }
});
