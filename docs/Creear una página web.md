## Creación de una página web en GitHub Pages

### 1) Resumen  
**Nombre del proyecto:** Publicación de página web con GitHub Pages  
**Equipo / Autor(es):** José Ismael Guerrero Román y Gerardo Esquivel De Luna  
**Curso / Asignatura:** Introducción a la Mecatrónica  
**Fecha:**   
**Descripción breve:**  
En esta práctica se creó y publicó una página web estática utilizando **GitHub Pages**, una herramienta gratuita que permite alojar sitios web directamente desde un repositorio en GitHub. Se aprendió a crear el repositorio, subir archivos HTML y configurar la publicación en línea.

---

### 2) Objetivos  
**General:**  
Publicar una página web básica en Internet utilizando GitHub Pages.  

**Específicos:**  
- OE1: Crear un repositorio en GitHub con los archivos del sitio web.  
- OE2: Configurar el repositorio para habilitar GitHub Pages.  
- OE3: Comprobar el funcionamiento del sitio publicado desde un navegador.  

---

### 3) Alcance y Exclusiones  
**Incluye:**  
- Creación de una página web estática (HTML, CSS y JS).  
- Publicación gratuita mediante GitHub Pages.  
- Visualización de la página desde cualquier navegador.  

**No incluye:**  
- Sitios con bases de datos o backend (solo contenido estático).  
- Integración de dominios personalizados.  
- Edición avanzada con frameworks o gestores de contenido.  

---

### 4) Requisitos  

**Software:**  
- Navegador web actualizado.  
- Cuenta en [GitHub](https://github.com).  
- (Opcional) Editor de código como Visual Studio Code.  

**Hardware:**  
- Computadora con conexión a Internet.  

**Conocimientos previos:**  
- Conceptos básicos de HTML y estructura de carpetas.  
- Manejo básico de GitHub (crear repositorios y subir archivos).  

---

### 5) Procedimiento paso a paso  

#### **Paso 1: Crear una cuenta en GitHub**
1. Ingresar a [https://github.com](https://github.com).  
2. Crear una cuenta nueva o iniciar sesión si ya se tiene una.  

---

#### **Paso 2: Crear un nuevo repositorio**
1. Desde el perfil, hacer clic en el botón **“New repository”**.  
2. Asignar un nombre al repositorio, por ejemplo:  
3. Marcar la opción **“Public”** y, si se desea, añadir un archivo **README.md**.  
4. Presionar **“Create repository”**.  

---

#### **Paso 3: Subir los archivos de la página**
1. Dentro del nuevo repositorio, hacer clic en **“Add file” → “Upload files”**.  
2. Seleccionar los archivos del sitio web, por ejemplo:
- `index.html`  
- `style.css`  
- `script.js`
3. Hacer clic en **“Commit changes”** para guardar los archivos.  

---

#### **Paso 4: Habilitar GitHub Pages**
1. Entrar a la pestaña **“Settings”** del repositorio.  
2. Desplazarse hacia abajo hasta la sección **“Pages”**.  
3. En **“Source”**, seleccionar la rama **`main`** y la carpeta **`/ (root)`**.  
4. Presionar **“Save”**.  

---

#### **Paso 5: Obtener la URL del sitio**
1. GitHub generará un enlace con el formato:
2. Esperar unos minutos a que se procese la publicación.  
3. Abrir el enlace en un navegador para visualizar la página en línea.  

---

### 6) Código de ejemplo (index.html)

```html
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Mi Primera Página con GitHub Pages</title>
<style>
 body {
   font-family: Arial, sans-serif;
   background-color: #f0f2f5;
   text-align: center;
   margin-top: 100px;
 }
 h1 {
   color: #005bbb;
 }
</style>
</head>
<body>
<h1>¡Hola Mundo!</h1>
<p>Esta es mi primera página web publicada en GitHub Pages.</p>
</body>
</html>
```
