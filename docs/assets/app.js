(function () {
  const pages = [
    {
      file: "content/01-getting-started.md",
      title: "入门与仓库地图",
      sections: ["intro", "quick-map", "learn-path", "build-run"],
    },
    {
      file: "content/02-ros-interface.md",
      title: "ROS 运行链路与消息接口",
      sections: ["ros-graph", "messages", "uav", "ugv", "external-position", "planner"],
    },
    {
      file: "content/03-core-packages.md",
      title: "重点包深入：common 与 UAV 控制",
      sections: ["deep-common", "deep-uav-control"],
    },
    {
      file: "content/04-tutorials.md",
      title: "重点包深入：sunray_tutorial",
      sections: ["deep-tutorial", "new-task"],
    },
    {
      file: "content/05-planner-ego.md",
      title: "规划工具与 EGO 接入",
      sections: ["deep-planner-utils", "deep-ego"],
    },
    {
      file: "content/06-scripts-ground-station.md",
      title: "快速启动脚本与地面站",
      sections: ["quick-scripts"],
    },
    {
      file: "content/07-extensions.md",
      title: "扩展模块",
      sections: ["vision", "formation", "gimbal-media", "communication", "fmt"],
    },
    {
      file: "content/08-debug-safety-appendix.md",
      title: "仿真、调试与安全速查",
      sections: ["simulation", "debug", "safety", "appendix"],
    },
  ];

  const content = document.getElementById("doc-content");
  const sectionToPage = new Map();
  const pageData = Array.isArray(window.SUNRAY_DOCS) ? window.SUNRAY_DOCS : [];
  const pageDataByFile = new Map(pageData.map((page) => [page.file, page.content]));
  let rendered = false;

  pages.forEach((page) => {
    page.sections.forEach((section) => sectionToPage.set(section, page));
  });

  marked.setOptions({
    gfm: true,
    breaks: false,
    mangle: false,
    headerIds: false,
  });

  function getCurrentSection() {
    const hash = decodeURIComponent(window.location.hash.replace(/^#/, ""));
    return sectionToPage.has(hash) ? hash : "intro";
  }

  function getCurrentPage() {
    return sectionToPage.get(getCurrentSection()) || pages[0];
  }

  function setLoading(page) {
    content.innerHTML = [
      '<div class="loading-panel">',
      "<h2>正在加载文档</h2>",
      `<p>正在读取 ${page ? page.title : "Markdown 内容"}...</p>`,
      "</div>",
    ].join("");
  }

  function setError(page, error) {
    content.innerHTML = [
      '<section class="load-error">',
      "<h2>文档加载失败</h2>",
      `<p>无法读取 <code>${page.file}</code>。</p>`,
      "<p>如果你正在双击打开 HTML，请确认 <code>assets/doc-data.js</code> 存在；如果通过静态服务打开，请确认 <code>content/</code> 下的 Markdown 文件路径没有改变。</p>",
      `<pre><code>${String(error && error.message ? error.message : error)}</code></pre>`,
      "</section>",
    ].join("");
  }

  async function loadPageMarkdown(page) {
    if (window.location.protocol === "file:") {
      const mirrored = pageDataByFile.get(page.file);
      if (mirrored == null) {
        throw new Error("file:// 模式缺少 doc-data.js 中的内容镜像");
      }
      return mirrored;
    }

    try {
      const response = await fetch(page.file, { cache: "no-cache" });
      if (!response.ok) {
        throw new Error(`${response.status} ${response.statusText}`);
      }
      return await response.text();
    } catch (error) {
      const mirrored = pageDataByFile.get(page.file);
      if (mirrored != null) {
        return mirrored;
      }
      throw error;
    }
  }

  async function loadAllMarkdown() {
    const chunks = [];
    for (const page of pages) {
      chunks.push(await loadPageMarkdown(page));
    }
    return chunks.join("\n\n");
  }

  function renderMarkdown(markdown) {
    content.innerHTML = marked.parse(markdown);
  }

  function scrollToSection(sectionId) {
    const target = document.getElementById(sectionId);
    if (!target) {
      return;
    }
    requestAnimationFrame(() => {
      target.scrollIntoView({ behavior: "smooth", block: "start" });
    });
  }

  function updateActiveNav(sectionId) {
    document.querySelectorAll(".sidebar nav a").forEach((link) => {
      link.classList.toggle("active", link.getAttribute("href") === `#${sectionId}`);
    });
  }

  async function route() {
    const sectionId = getCurrentSection();
    const page = getCurrentPage();
    updateActiveNav(sectionId);

    try {
      if (!rendered) {
        setLoading(page);
        const markdown = await loadAllMarkdown();
        renderMarkdown(markdown);
        rendered = true;
      }
      scrollToSection(sectionId);
    } catch (error) {
      setError(page, error);
    }
  }

  window.addEventListener("hashchange", route);
  document.addEventListener("DOMContentLoaded", route);
})();
